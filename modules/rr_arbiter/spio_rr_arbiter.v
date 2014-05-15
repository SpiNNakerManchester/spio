/**
 * A round-robbin packet arbiter.
 */


module spio_rr_arbiter#( // The size of individual packets.
                         parameter PKT_BITS = 72
                       )
                       ( input wire CLK_IN
                       , input wire RESET_IN
                         // Input ports
                       , input  wire [PKT_BITS-1:0] DATA0_IN
                       , input  wire                VLD0_IN
                       , output wire                RDY0_OUT
                       , input  wire [PKT_BITS-1:0] DATA1_IN
                       , input  wire                VLD1_IN
                       , output wire                RDY1_OUT
                         // Output port where the merged stream will be sent
                       , output reg  [PKT_BITS-1:0] DATA_OUT
                       , output reg                 VLD_OUT
                       , input  wire                RDY_IN
                       );

// The following sketch outlines the datapath and control signals.
//
//               park0_i
//                  | ,---,
//                  '-|En | parkreg0_i
//               ,----|D Q|----,  |\
//               |    |>  |    '--|1|       data0_i
//               |    '---'       | |----------------------,
//   DATA0_IN  --+----------------|0|                      |
//                                |/                       |
//                                 '--- parked0_i          |
//                    --+--                                |
//                      |         |\                       |
//                      '---------|1|                      |
//                                | |-- vld0_i             |
//    VLD0_IN  -------------------|0|                      |
//                                |/                       |
//                                 '--- parked0_i          | selected1_i
//                                                         |    |
//   RDY0_OUT  --------------                              |    |
//                                                         |    | cansend_i
//                                                         |   |\   | ,---,
//                                                         '---|0|  '-|En |
//   - - - - - - - - - - - - - - - - - - - - - - - -           | |----|D Q|---- DATA_OUT
//                                                         ,---|1|    |>  |
//               park1_i                                   |   |/     '---'
//                  | ,---,                                |
//                  '-|En | parkreg1_i                     |      cansend_i
//               ,----|D Q|----,  |\                       |        | ,---,
//               |    |>  |    '--|1|       data1_i        |        '-|En |
//               |    '---'       | |----------------------'        --|D Q|---- VLD_OUT
//   DATA1_IN  --+----------------|0|                                 |>  |
//                                |/                                  '---'
//                                 '--- parked1_i
//                    --+--                                                ---- RDY_IN
//                      |         |\
//                      '---------|1|
//                                | |-- vld1_i
//    VLD1_IN  -------------------|0|
//                                |/
//                                 '--- parked1_i
//
//   RDY1_OUT  --------------
//

// State values for input parked FSMs
localparam RUN    = 1'b0;
localparam PARKED = 1'b1;

reg inputstate0_i;
reg inputstate1_i;

// State values for round-robbin priority selection FSM
localparam P0 = 1'b0;
localparam P1 = 1'b1;

reg rrstate_i;

// Parking registers
reg [PKT_BITS-1:0] parkreg0_i;
reg [PKT_BITS-1:0] parkreg1_i;

// Signal indicating that a new value may be sent to the output port (e.g. if it
// is idle or is ready)
wire cansend_i = !VLD_OUT || RDY_IN;

// Signal indicating whether a value is being transferred on the output port
// this cycle
wire transfer_i = VLD_OUT && RDY_IN;

// Signal indicating if an input has valid data. Note that if parked then there
// is valid data in the parkreg*_i register.
wire vld0_i = (inputstate0_i == PARKED) ? 1'b1 : VLD0_IN;
wire vld1_i = (inputstate1_i == PARKED) ? 1'b1 : VLD1_IN;

// The data to be transferred from each input. This is either the incoming data
// or the parked data if present.
wire [PKT_BITS-1:0] data0_i = (inputstate0_i == PARKED) ? parkreg0_i : DATA0_IN;
wire [PKT_BITS-1:0] data1_i = (inputstate1_i == PARKED) ? parkreg1_i : DATA1_IN;

// Signals indicating if a given input was selected by the arbiter. If the
// output port isn't blocked then this input's value will be sent in the next
// cycle. The input selected by the round-robbin priority FSM will be selected
// unless it has no data to transfer in which case the other input may go.
wire selected0_i = rrstate_i == P0 || !vld1_i;
wire selected1_i = rrstate_i == P1 || !vld0_i;

// An input must be parked when it is valid if either the output is blocked or
// the input has not been selected to output.
wire wait0_i = !cansend_i || !selected0_i;
wire wait1_i = !cansend_i || !selected1_i;

// Signal which is asserted when an incoming value must be parked.
wire park0_i = VLD0_IN && RDY0_OUT && wait0_i;
wire park1_i = VLD1_IN && RDY1_OUT && wait1_i;

// Signal which is asserted whenever an input's value can be accepted.
wire go0_i = !wait0_i;
wire go1_i = !wait1_i;


////////////////////////////////////////////////////////////////////////////////
// Input ready signals
////////////////////////////////////////////////////////////////////////////////

// Inputs are guarunteed to be able to accept a packet whenever they're not in
// the parked state: if their output is free, the packet will be forwarded
// immediately, if not the parking register will be used.

assign RDY0_OUT = inputstate0_i == RUN;
assign RDY1_OUT = inputstate1_i == RUN;


////////////////////////////////////////////////////////////////////////////////
// Parking registers
////////////////////////////////////////////////////////////////////////////////

always @ (posedge CLK_IN)
	if (park0_i)
		parkreg0_i <= DATA0_IN;

always @ (posedge CLK_IN)
	if (park1_i)
		parkreg1_i <= DATA1_IN;


////////////////////////////////////////////////////////////////////////////////
// Output registers
////////////////////////////////////////////////////////////////////////////////

// If the output is idle (i.e. not valid) or it is ready, forward the selected
// input if it is valid, otherwise output nothing.
always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		begin
			DATA_OUT <= {PKT_BITS{1'bX}};
			VLD_OUT  <= 1'b0;
		end
	else
		if (cansend_i)
			begin
				if (selected0_i && vld0_i && (RDY0_OUT || inputstate0_i == PARKED))
					begin
						DATA_OUT <= data0_i;
						VLD_OUT  <= vld0_i;
					end
				else if (selected1_i && vld1_i && (RDY1_OUT || inputstate1_i == PARKED))
					begin
						DATA_OUT <= data1_i;
						VLD_OUT  <= vld1_i;
					end
				else
					begin
						DATA_OUT <= {PKT_BITS{1'bX}};
						VLD_OUT  <= 1'b0;
					end
			end


////////////////////////////////////////////////////////////////////////////////
// Input parked control FSMs
////////////////////////////////////////////////////////////////////////////////

//         go0_i            wait0_i              go1_i            wait1_i
//         ,--,  park0_i     ,--,                ,--,  park1_i     ,--,
//         |  | ,-------,    |  |                |  | ,-------,    |  |
//         |  V |       V    |  V                |  V |       V    |  V
//        ,------,     ,---------,              ,------,     ,---------,
//   ---> | RUN0 |     | PARKED0 |         ---> | RUN1 |     | PARKED1 |
//        '------'     '---------'              '------'     '---------'
//              ^       |                             ^       |
//              '-------'                             '-------'
//                go0_i                                 go1_i

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		inputstate0_i <= RUN;
	else
		case (inputstate0_i)
			RUN:    if (park0_i)  inputstate0_i <= PARKED;
			PARKED: if (go0_i)    inputstate0_i <= RUN;
		endcase

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		inputstate1_i <= RUN;
	else
		case (inputstate1_i)
			RUN:    if (park1_i)  inputstate1_i <= PARKED;
			PARKED: if (go1_i)    inputstate1_i <= RUN;
		endcase


////////////////////////////////////////////////////////////////////////////////
// Priority control FSM
////////////////////////////////////////////////////////////////////////////////

// Simply alternate priority between the two input ports whenever a value leaves
// the output.
//
//           transfer_i
//            ,-----,
//            |     V
//        ,----,   ,----,
//   ---> | P0 |   | P1 |
//        '----'   '----'
//            ^     |
//            '-----'
//           transfer_i

always @ (posedge CLK_IN, posedge RESET_IN)
	if (RESET_IN)
		rrstate_i <= P0;
	else
		if (transfer_i)
			case (rrstate_i)
				P0: rrstate_i <= P1;
				P1: rrstate_i <= P0;
			endcase



endmodule
