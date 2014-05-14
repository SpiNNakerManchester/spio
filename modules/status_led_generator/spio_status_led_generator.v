/**
 * Generate PWM LED driving signals to indicate system status for a number of
 * devices.
 */

module spio_status_led_generator #( // The number of devices (and thus LEDs)
                                    parameter NUM_DEVICES = 1
                                    // Animation period in clock cycles
                                  , parameter ANIMATION_PERIOD_BITS = 27
                                    // Duration of brief pulses (cycles)
                                  , parameter PULSE_DURATION = 7500000
                                    // Which bit of the period counter should be
                                    // used to produce the activity blink
                                  , parameter ACTIVITY_BLINK_BIT =  23
                                    // Number of bits PWM resolution
                                  , parameter PWM_BITS = 7
                                    // Timeout for non-activity before
                                    // deasserting the activity status.
                                  , parameter ACTIVITY_TIMEOUT = 37500000
                                  , parameter ACTIVITY_TIMEOUT_BITS = 26
                                  )
                                  ( input wire CLK_IN
                                  , input wire RESET_IN
                                    // Active-high error signal (one per device)
                                  , input wire [NUM_DEVICES-1:0] ERROR_IN
                                    // Active-high "connected" signal (one per device)
                                  , input wire [NUM_DEVICES-1:0] CONNECTED_IN
                                    // Active-high activity signal (one per device)
                                  , input wire [NUM_DEVICES-1:0] ACTIVITY_IN
                                    // The LED state (one per device)
                                  , output reg [NUM_DEVICES-1:0] LED_OUT
                                    // Produces a one-cycle pulse at the end of
                                    // every animation loop.
                                  , output reg ANIMATION_REPEAT_OUT
                                  );
genvar i;

// Current PWM duty cycle (note: one bit larger than the PWM counter)
reg [PWM_BITS:0] pwm_value_i [NUM_DEVICES-1:0];

// Whenever there has been some activity within the last ACTIVITY_TIMEOUT
// cycles, this signal is asserted.
wire activity_i [NUM_DEVICES-1:0];

// Animations
reg [PWM_BITS-1:0] animation_pulse_i;
reg [PWM_BITS-1:0] animation_blink_i;
reg [PWM_BITS-1:0] animation_throb_i;

////////////////////////////////////////////////////////////////////////////////
// PWM generator
////////////////////////////////////////////////////////////////////////////////

// Free-running PWM timer
reg [PWM_BITS-1:0] pwm_counter_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		pwm_counter_i <= 0;
	else
		pwm_counter_i <= pwm_counter_i + 1;


// Comparator for each LED
generate for (i = 0; i < NUM_DEVICES; i = i + 1)
	begin : pwm_comparator
		always @ (posedge CLK_IN)
			LED_OUT[i] <= pwm_counter_i < pwm_value_i[i];
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Animation generation
////////////////////////////////////////////////////////////////////////////////

// The current time in the animation cycle
reg [ANIMATION_PERIOD_BITS-1:0] period_i;
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		period_i <= 0;
	else
		period_i <= period_i + 1;


// End of animation period (since all animations are one cycle behind the
// period counter)
always @ (posedge CLK_IN)
	ANIMATION_REPEAT_OUT <= period_i == 0;


// Brief pulsing animation (with the pulse in the middle of the animation)
always @ (posedge CLK_IN)
	animation_pulse_i <= {PWM_BITS { (period_i-{1'b1, {ANIMATION_PERIOD_BITS-1{1'b0}}})
	                                 < PULSE_DURATION
	                               }
	                     };

// Activity blinking animation
always @ (posedge CLK_IN)
	animation_blink_i <= {PWM_BITS{period_i[ACTIVITY_BLINK_BIT]}};

// Throbbing
always @ (posedge CLK_IN)
	if (period_i[ANIMATION_PERIOD_BITS-1] == 1'b0)
		// Fade on for first half...
		animation_throb_i <= period_i[ ANIMATION_PERIOD_BITS-2
		                             : ANIMATION_PERIOD_BITS-PWM_BITS-1
		                             ];
	else
		// ...and off for second half.
		animation_throb_i <= {PWM_BITS{1'b1}} - period_i[ ANIMATION_PERIOD_BITS-2
		                                                : ANIMATION_PERIOD_BITS-PWM_BITS-1
		                                                ];


////////////////////////////////////////////////////////////////////////////////
// Activity timeout
////////////////////////////////////////////////////////////////////////////////

generate for (i = 0; i < NUM_DEVICES; i = i + 1)
	begin : activity_timeouts
		reg [ACTIVITY_TIMEOUT_BITS-1:0] timeout_i;
		
		always @ (posedge CLK_IN, posedge RESET_IN)
			if (RESET_IN)
				timeout_i <= 0;
			else
				if (ACTIVITY_IN[i])
					timeout_i <= ACTIVITY_TIMEOUT;
				else if (timeout_i != 0)
					timeout_i <= timeout_i - 1;
		
		assign activity_i[i] = timeout_i != 0;
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////
// Animation selection
////////////////////////////////////////////////////////////////////////////////

generate for (i = 0; i < NUM_DEVICES; i = i + 1)
	begin : animation_selection
		always @ (posedge CLK_IN)
			casex ({ERROR_IN[i], CONNECTED_IN[i], activity_i[i]})
				3'b1xx:  pwm_value_i[i] <= ~animation_pulse_i;
				3'b011:  pwm_value_i[i] <= animation_blink_i;
				3'b010:  pwm_value_i[i] <= animation_throb_i;
				default: pwm_value_i[i] <= animation_pulse_i;
			endcase
	end
endgenerate

endmodule
