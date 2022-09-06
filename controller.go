package pid

import (
	"context"
	"fmt"
	"strings"
	"sync"
	"time"
)

// ControllerConfig is the configuration you want for a PID controller.
//
// The proportional gain controls response to the error (set point - measured value) at the time of measurement.
//
// The integral gain controls response to the accumulated error over time.
//
// The derivative gain controls response to the change in error between the last measurement time and the current measurement time.
//
// The set point is the desired value you want for whatever it is you are measuring in order to establish control.
//
// The measurement period is how often you want the measurements and adjustments to happen.
//
// The MeasureValueFunction is a function provided by you to measure whatever it is you are using to determine performance of whatever you are having the PID controller monitor.
// There are a lot of measurement techniques and principles out there.
// A feedforward controller can be made by having a predicted result you provide that will adjust the current measured value.
// For example, if you have predictable trends on your system, you can use that to supplement the measured value to insure you are within desirable bounds for your known load.
//
// The HandleMeasurementErrorFunction is a function provided by you to handle errors returned by your MeasureValueFunction.
// If a function is not provided, then the controller will default to simply just ignoring the error.
// If an error does occur, the calculation of control adjustment, and application thereof, does not occur.
//
// The ApplyControlFunction is a function provided by you in order to facilitate taking the control adjustment value provided by the controller and applying it to whatever you are trying to control with the PID controller.
// If it takes time for the application of the control adjustment to have real effects in the system, then it could be beneficial for your function to wait for the real effect to happen before returning.
// This will have the controller handle the jump in time better since it won't be taking measurements in the time the system was reacting to the adjustment.
//
// The HandleApplyControlErrorFunction is another function provided by you in order to handle errors returned by your ApplyControlFunction.
// If it is not specified, the controller will default to just ignoring the errors.
type ControllerConfig struct {
	Settings                        *ControllerSettings
	MeasureValueFunction            func() (float64, error)
	HandleMeasurementErrorFunction  func(error)
	ApplyControlFunction            func(float64) error
	HandleApplyControlErrorFunction func(error)
}

type ControllerSettings struct {
	ProportionalGain  float64
	IntegralGain      float64
	DerivativeGain    float64
	SetPoint          float64
	MeasurementPeriod time.Duration
}

type Controller struct {
	settings               *ControllerSettings
	measureValue           func() (float64, error)
	handleMeasurementError func(error)
	lastMeasurementTime    time.Time
	timeOfNextMeasurement  time.Time
	applyControl           func(float64) error
	handleControlError     func(error)
	integralErrorAmount    float64
	previousErrorAmount    float64
	sync.Mutex
}

// NewController uses the given config to create a new PID controller.
// The ApplyControlFunction and MeasureValueFunctions must be provided.
// If they are not, an error is returned.
// If HandleMeasurementErrorFunction is not defined, it will be defaulted to a do nothing function which will effectively completely ignore the error, likewise for HandleApplyControlErrorFunction.
func NewController(config *ControllerConfig) (Controller, error) {
	missingFields := make([]string, 0)
	if config.ApplyControlFunction == nil {
		missingFields = append(missingFields, "ApplyControlFunction")
	}
	if config.MeasureValueFunction == nil {
		missingFields = append(missingFields, "MeasureValueFunction")
	}
	if len(missingFields) > 0 {
		return Controller{}, fmt.Errorf("one or more required config values not specified: %s", strings.Join(missingFields, ","))
	}
	if config.HandleApplyControlErrorFunction == nil {
		config.HandleApplyControlErrorFunction = func(error) {}
	}
	if config.HandleMeasurementErrorFunction == nil {
		config.HandleMeasurementErrorFunction = func(error) {}
	}
	return Controller{
		settings:               config.Settings,
		handleControlError:     config.HandleApplyControlErrorFunction,
		applyControl:           config.ApplyControlFunction,
		measureValue:           config.MeasureValueFunction,
		handleMeasurementError: config.HandleMeasurementErrorFunction,
	}, nil
}

// Monitor is used to start the PID controller.
// This function blocks until the given context is done.
// It will take a measurement using the MeasureValueFunction provided when the controller was created.
// If the function returns an error, then the HandleMeasureValueErrorFunction is used to handle that error, and the calculation of error and control adjustment is skipped in the case.
// If there is no error, then the controller calculates an adjustment to be made to the system and uses this to call the provided ApplyControlFunction.
// If the provided ApplyControlFunction returns an error, then the HandleApplyControlErrorFunction is called to handle the error.
// The state of the controller is updated with the new information:
// the previous error in the controller is updated to be the error at time of measurement,
// the time for the next measurement is updated to be the time of last measurement plus the measurement period,
// and the time of last measurement is updated to the time of measurement this cycle.
func (controller *Controller) Monitor(ctx context.Context) error {
	controller.lastMeasurementTime = time.Now()
	controller.timeOfNextMeasurement = controller.lastMeasurementTime.Add(controller.settings.MeasurementPeriod)
	for {
		select {
		case <-time.After(controller.timeOfNextMeasurement.Sub(time.Now())):
			controller.Lock()
			timeNow := time.Now()
			measuredValue, err := controller.measureValue()
			if err != nil {
				controller.handleMeasurementError(err)
			} else {
				currentError := controller.settings.SetPoint - measuredValue
				timeDifference := float64(timeNow.Sub(controller.lastMeasurementTime))
				controller.integralErrorAmount = controller.integralErrorAmount + (currentError * timeDifference)
				derivativeErrorAmount := (currentError - controller.previousErrorAmount) / timeDifference
				controlAdjustment := currentError*controller.settings.ProportionalGain + controller.integralErrorAmount*controller.settings.IntegralGain + derivativeErrorAmount*controller.settings.DerivativeGain
				err = controller.applyControl(controlAdjustment)
				if err != nil {
					controller.handleControlError(err)
				}
				controller.previousErrorAmount = currentError
			}
			controller.timeOfNextMeasurement = controller.lastMeasurementTime.Add(controller.settings.MeasurementPeriod)
			controller.lastMeasurementTime = timeNow
			controller.Unlock()
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// AdjustSettings will make the controller settings be the new settings given.
// Adjustments will need to be made upon first running the controller.
// Adjustments may also need to be made if the fundamental operating nature of the monitored system changes.
// If you begin to experience erratic behavior of the monitored system, then adjustments are probably needed.
func (controller *Controller) AdjustSettings(newSettings *ControllerSettings) {
	controller.Lock()
	defer controller.Unlock()
	controller.settings = newSettings
}

// GetCurrentSettings returns a manifest of the current settings.
func (controller *Controller) GetCurrentSettings() ControllerSettings {
	controller.Lock()
	defer controller.Unlock()
	return ControllerSettings{
		ProportionalGain:  controller.settings.ProportionalGain,
		IntegralGain:      controller.settings.IntegralGain,
		DerivativeGain:    controller.settings.DerivativeGain,
		SetPoint:          controller.settings.SetPoint,
		MeasurementPeriod: controller.settings.MeasurementPeriod,
	}
}
