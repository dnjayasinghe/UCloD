/*-------------------------------------------------------------------------
 AES cryptographic module for FPGA on SASEBO-GIII
 
 File name   : chip_sasebo_giii_aes.v
 Version     : 1.0
 Created     : APR/02/2012
 Last update : APR/25/2013
 Desgined by : Toshihiro Katashita
 
 
 Copyright (C) 2012,2013 AIST
 
 By using this code, you agree to the following terms and conditions.
 
 This code is copyrighted by AIST ("us").
 
 Permission is hereby granted to copy, reproduce, redistribute or
 otherwise use this code as long as: there is no monetary profit gained
 specifically from the use or reproduction of this code, it is not sold,
 rented, traded or otherwise marketed, and this copyright notice is
 included prominently in any copy made.
 
 We shall not be liable for any damages, including without limitation
 direct, indirect, incidental, special or consequential damages arising
 from the use of this code.
 
 When you publish any results arising from the use of this code, we will
 appreciate it if you can cite our paper.
 (http://www.risec.aist.go.jp/project/sasebo/)
 -------------------------------------------------------------------------*/


//================================================ CHIP_SASEBO_GIII_AES
module CHIP_SASEBO_GIII_AES
  (// Local bus for GII
   lbus_di_a, lbus_do, lbus_wrn, lbus_rdn,
   lbus_clkn, lbus_rstn,

   // GPIO and LED
   gpio_startn, gpio_endn, gpio_exec, led,

   // Clock OSC
   osc_en_b,
	tx,
	clkOut);
   
   //------------------------------------------------
   // Local bus for GII
   input [15:0]  lbus_di_a;
   output [15:0] lbus_do;
   input         lbus_wrn, lbus_rdn;
   input         lbus_clkn, lbus_rstn;

   // GPIO and LED
   output        gpio_startn, gpio_endn, gpio_exec;
   output [9:0]  led;

   // Clock OSC
   output        osc_en_b;
	
	// UART
	output tx;
	//output clk;
	
	output clkOut;
	
	//output clko;
   //------------------------------------------------
   // Internal clock
   wire         clk, rst;
	//NET "clkOutP" LOC = H17 |IOSTANDARD = LVCMOS25; 
   //assign clko = clk2;
	
   // Local bus
   reg [15:0]   lbus_a, lbus_di;
   
   // Block cipher
   wire [127:0] blk_kin, blk_din, blk_dout;
   wire         blk_krdy, blk_kvld, blk_drdy, blk_dvld;
   wire         blk_encdec, blk_en, blk_rstn, blk_busy;
   reg          blk_drdy_delay;
  
   wire clk0, clk1, clk3;
	reg startTx;
	//reg [7:0]  data;
	wire busyTX, Tx, doneTx;
   
	
	
	//wire clkAA, clkA;
 //  reg [7:0]power [1023:0];
  
  //assign gnd = GND;
  
   //------------------------------------------------
   assign led[0] = rst;
   assign led[1] = lbus_rstn;
   assign led[2] = 1'b0;
   assign led[3] = blk_rstn;
   assign led[4] = blk_encdec;
   assign led[5] = blk_krdy;
   assign led[6] = blk_kvld;
   assign led[7] = 1'b0;
   assign led[8] = blk_dvld;
   assign led[9] = blk_busy;
   assign osc_en_b = 1'b1;
   //------------------------------------------------
   always @(posedge clk) if (lbus_wrn)  lbus_a  <= lbus_di_a;
   always @(posedge clk) if (~lbus_wrn) lbus_di <= lbus_di_a;

   LBUS_IF lbus_if
     (.lbus_a(lbus_a), .lbus_di(lbus_di), .lbus_do(lbus_do),
      .lbus_wr(lbus_wrn), .lbus_rd(lbus_rdn),
      .blk_kin(blk_kin), .blk_din(blk_din), .blk_dout(blk_dout),
      .blk_krdy(blk_krdy), .blk_drdy(blk_drdy), 
      .blk_kvld(blk_kvld), .blk_dvld(blk_dvld),
      .blk_encdec(blk_encdec), .blk_en(blk_en), .blk_rstn(blk_rstn),
      .clk(clk), .rst(rst));

   //------------------------------------------------
   assign gpio_startn = ~blk_drdy;
   assign gpio_endn   = 1'b0; //~blk_dvld;
   assign gpio_exec   = 1'b0; //blk_busy;

   //always @(posedge clk) blk_drdy_delay <= blk_drdy;
//
//   AES_Composite_enc AES_Composite_enc
//     (.Kin(blk_kin), .Din(blk_din), .Dout(blk_dout),
//      .Krdy(blk_krdy), .Drdy(blk_drdy), .Kvld(blk_kvld), .Dvld(blk_dvld),
//      .EncDec(blk_encdec), .EN(blk_en), .BSY(blk_busy),
//      .CLK(clk), .RSTn(blk_rstn));

	//uart_tx uart_tx(.i_Clock(clk), .i_Tx_DV(startTX), .i_Tx_Byte(data), .o_Tx_Active(busyTx), .o_Tx_Serial(Tx), .o_Tx_Done(doneTx));

   //------------------------------------------------   
   MK_CLKRST mk_clkrst (.clkin(clk), .rstnin(lbus_rstn),
                        .clk(), .rst(rst),.clk00(), .clk11(), .clk22());
	wire clkO;	
   wire   refclk, clkA, clkAA, clkAAA, clk5, clk5t, clk2t;
	assign clkOut = clk2;
//
////(*LOC="ODELAY_X0Y1"*)
//(* IODELAY_GROUP = "Group_ADS1" *)
//ODELAYE2 #(
//.CINVCTRL_SEL("FALSE"), // Enable dynamic clock inversion (FALSE, TRUE)
//.DELAY_SRC("ODATAIN"), // Delay input (IDATAIN, DATAIN)
//.HIGH_PERFORMANCE_MODE("FALSE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
//.ODELAY_TYPE("FIXED"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
//.ODELAY_VALUE(0), // Input delay tap setting (0-31)
//.PIPE_SEL("FALSE"), // Select pipelined mode, FALSE, TRUE
//.REFCLK_FREQUENCY(200.0), // IDELAYCTRL clock input frequency in MHz (190.0-210.0).
//.SIGNAL_PATTERN("CLOCK") // DATA, CLOCK input signal
//)
//ODELAYE2_inst0 (
//.CNTVALUEOUT(), // 5-bit output: Counter value output
//.DATAOUT(clkA), // 1-bit output: Delayed data output
//.C(1'b0), // 1-bit input: Clock input  refclk
//.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
//.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
//.CNTVALUEIN(5'b11111), // 5-bit input: Counter value input
////.DATAIN(clk0Out), // 1-bit input: Internal delay data input
////.CLKIN(refclk1), // 1-bit input: Internal delay data input
//.ODATAIN(refclk1), // 1-bit input: Data input from the I/O
//.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
//.LD(1'b0), // 1-bit input: Load IDELAY_VALUE input
//.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
//.REGRST(~lbus_rstn) // 1-bit input: Active-high reset tap-delay input
//);
//
reg [127:0] rnd1;
reg [127:0] rnd2;
reg [127:0] rnd3;
reg [127:0] rnd4;
reg [4:0] r1;
reg [4:0] r2;
reg [4:0] r3;	

	
//(*LOC="IDELAY_X1Y1"*)
(* IODELAY_GROUP = "Group_ADS1" *)
IDELAYE2 #(
.CINVCTRL_SEL("FALSE"), // Enable dynamic clock inversion (FALSE, TRUE)
.DELAY_SRC("DATAIN"), // Delay input (IDATAIN, DATAIN)
.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
.IDELAY_TYPE("VAR_LOAD_PIPE"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
.IDELAY_VALUE(0), // Input delay tap setting (0-31)
.PIPE_SEL("TRUE"), // Select pipelined mode, FALSE, TRUE
.REFCLK_FREQUENCY(192.0), // IDELAYCTRL clock input frequency in MHz (190.0-210.0).
.SIGNAL_PATTERN("CLOCK") // DATA, CLOCK input signal
)
IDELAYE2_inst0 (
.CNTVALUEOUT(), // 5-bit output: Counter value output
.DATAOUT(clkAA), // 1-bit output: Delayed data output
.C(refclk1), // 1-bit input: Clock input  refclk
.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
.CNTVALUEIN(rnd1[4:0]), // 5-bit input: Counter value input
.DATAIN(refclk1), // 1-bit input: Internal delay data input
.IDATAIN(), // 1-bit input: Data input from the I/O
.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
.LD(1'b1), // 1-bit input: Load IDELAY_VALUE input
.LDPIPEEN(1'b1), // 1-bit input: Enable PIPELINE register to load data input
.REGRST(~lbus_rstn) // 1-bit input: Active-high reset tap-delay input
);


(* IODELAY_GROUP = "Group_ADS1" *)
IDELAYE2 #(
.CINVCTRL_SEL("FALSE"), // Enable dynamic clock inversion (FALSE, TRUE)
.DELAY_SRC("DATAIN"), // Delay input (IDATAIN, DATAIN)
.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
.IDELAY_TYPE("VAR_LOAD_PIPE"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
.IDELAY_VALUE(0), // Input delay tap setting (0-31)
.PIPE_SEL("FALSE"), // Select pipelined mode, FALSE, TRUE
.REFCLK_FREQUENCY(192.0), // IDELAYCTRL clock input frequency in MHz (190.0-210.0).
.SIGNAL_PATTERN("CLOCK") // DATA, CLOCK input signal
)
IDELAYE2_inst1 (
.CNTVALUEOUT(), // 5-bit output: Counter value output
.DATAOUT(clkAAA), // 1-bit output: Delayed data output
.C(refclk1), // 1-bit input: Clock input  refclk
.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
.CNTVALUEIN(rnd1[36:32]), // 5-bit input: Counter value input
.DATAIN(clkAA), // 1-bit input: Internal delay data input
.IDATAIN(), // 1-bit input: Data input from the I/O
.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
.LD(1'b1), // 1-bit input: Load IDELAY_VALUE input
.LDPIPEEN(1'b1), // 1-bit input: Enable PIPELINE register to load data input
.REGRST(~lbus_rstn) // 1-bit input: Active-high reset tap-delay input
);


(* IODELAY_GROUP = "Group_ADS1" *)
IDELAYE2 #(
.CINVCTRL_SEL("TRUE"), // Enable dynamic clock inversion (FALSE, TRUE)
.DELAY_SRC("DATAIN"), // Delay input (IDATAIN, DATAIN)
.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
.IDELAY_TYPE("VAR_LOAD_PIPE"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
.IDELAY_VALUE(0), // Input delay tap setting (0-31)
.PIPE_SEL("TRUE"), // Select pipelined mode, FALSE, TRUE
.REFCLK_FREQUENCY(192.0), // IDELAYCTRL clock input frequency in MHz (190.0-210.0).
.SIGNAL_PATTERN("CLOCK") // DATA, CLOCK input signal
)
IDELAYE2_inst2 (
.CNTVALUEOUT(), // 5-bit output: Counter value output
.DATAOUT(clkA), // 1-bit output: Delayed data output
.C(refclk1), // 1-bit input: Clock input  refclk
.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
.CNTVALUEIN(rnd1[68:64]), // 5-bit input: Counter value input
.DATAIN(clkAAA), // 1-bit input: Internal delay data input
.IDATAIN(), // 1-bit input: Data input from the I/O
.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
.LD(1'b1), // 1-bit input: Load IDELAY_VALUE input
.LDPIPEEN(1'b1), // 1-bit input: Enable PIPELINE register to load data input
.REGRST(~lbus_rstn) // 1-bit input: Active-high reset tap-delay input
);




reg i;
reg [31:0] m1;
reg [31:0] d1;



parameter a=6;
parameter b=4;
parameter k=4;

always @ (posedge refclk1) begin

	if(lbus_rstn==0) begin
	//m1 = randi([0,(a-b)*pow2(k)],1);
	m1 <= 16;//(a-b)*rnd1[k:0]
	rnd1<=128'hab65272819bac345;
	//rnd2<=128'habbf2abcff1ad325;
	end
	else begin
		rnd1 <=  {rnd1[126:0],rnd1[127]^~rnd1[125]^~rnd1[100]^~rnd1[98]}; 
		//rnd2 <=  {rnd2[126:0],rnd2[127]^~rnd2[125]^~rnd2[100]^~rnd2[98]};
		end

end



//
//
//always @(posedge refclk1) begin
//
//
//	
//	if (lbus_rstn==0) begin
//	rnd1<=128'hab65272819bac345;
//	rnd2<=128'habbf2abcff1ad325;
//	rnd3<=128'hab7f2aa865892292;
//	//rnd4<=128'hab512abc65abfa92;
//	end  
//	else begin
//	rnd1 <=  {rnd1[126:0],rnd1[127]^~rnd1[125]^~rnd1[100]^~rnd1[98]};  //128,126,101,99
//	rnd2 <=  {rnd2[126:0],rnd2[127]^~rnd2[125]^~rnd2[100]^~rnd2[98]}; //128,126,101,99
//	rnd3 <=  {rnd3[126:0],rnd3[127]^~rnd3[125]^~rnd3[100]^~rnd3[98]}; //128,126,101,99
//	//rnd4 <=  {rnd4[126:0],rnd4[127]^~rnd4[125]^~rnd4[100]^~rnd4[98]}; //128,126,101,99
//	end
//
//
//	//rnd <= rnd+1;
//
//end

//assign clkRN = ~clkR;

//(*LOC="IDELAY_X1Y1"*)
(* IODELAY_GROUP = "Group_ADS1" *)
IDELAYCTRL IDELAYCTRL_inst (
.RDY(), // 1-bit output: Ready output
.REFCLK(clk2), // 1-bit input: Reference clock input  //CLKOUTREF
.RST(~lbus_rstn) // 1-bit input: Active high reset input
);

	BUFG u50 (.I(clk0), .O(refclk1)); 
	BUFG u51 (.I(clkA), .O(clk)); 
	BUFG u52 (.I(clk2t), .O(clk2)); 
	BUFG u53 (.I(clk5t), .O(clk5)); 

wire CLKFBOUT1, CLKFBIN1,Cout;

 BUFG  u25 (.I(CLKFBOUT1),   .O(CLKFBIN1));
 BUFG  u26 (.I(clk3),   .O(clk21));
//(*LOC="MMCME2_ADV_X1Y2"*)
MMCME2_ADV #(
		.BANDWIDTH("OPTIMIZED"), // Jitter programming (OPTIMIZED, HIGH, LOW)
		.CLKFBOUT_MULT_F(5.0), // Multiply value for all CLKOUT (2.000-64.000).
		.CLKFBOUT_PHASE(0.0), // Phase offset in degrees of CLKFB (-360.000-360.000).
		.CLKIN1_PERIOD(5.208), // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
		// CLKOUT0_DIVIDE - CLKOUT6_DIVIDE: Divide amount for each CLKOUT (1-128)
		.CLKOUT1_DIVIDE(40),
		.CLKOUT2_DIVIDE(1),
		.CLKOUT3_DIVIDE(1),
		.CLKOUT4_DIVIDE(1),
		.CLKOUT5_DIVIDE(1),
		.CLKOUT6_DIVIDE(1),
		.CLKOUT0_DIVIDE_F(5.0), // Divide amount for CLKOUT0 (1.000-128.000).
		// CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for each CLKOUT (0.01-0.99).
		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),
		.CLKOUT6_DUTY_CYCLE(0.5),
		// CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
		.CLKOUT0_PHASE(0.0),
		.CLKOUT1_PHASE(0.0),
		.CLKOUT2_PHASE(0.0),
		.CLKOUT3_PHASE(0.0),
		.CLKOUT4_PHASE(0.0),
		.CLKOUT5_PHASE(0.0),
		.CLKOUT6_PHASE(0.0),
		.CLKOUT4_CASCADE("FALSE"), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
		.DIVCLK_DIVIDE(1), // Master division value (1-106)
		.REF_JITTER1(0.0), // Reference input jitter in UI (0.000-0.999).
		.STARTUP_WAIT("FALSE") // Delays DONE until MMCM is locked (FALSE, TRUE)
	)
	MMCME2_BASE_inst1 (
		// Clock Outputs: 1-bit (each) output: User configurable clock outputs
		.CLKOUT0(clk2t), // 1-bit output: CLKOUT0
		.CLKOUT1(clk5t), // 1-bit output: CLKOUT1
		.CLKOUT2(), // 1-bit output: CLKOUT2
		//.CLKOUT3(CLKOUT3), // 1-bit output: CLKOUT3
		//.CLKOUT4(CLKOUT4), // 1-bit output: CLKOUT4
		//.CLKOUT5(CLKOUT5), // 1-bit output: CLKOUT5
		//.CLKOUT6(), // 1-bit output: CLKOUT6
		// Feedback Clocks: 1-bit (each) output: Clock feedback ports
		.CLKFBOUT(CLKFBOUT1), // 1-bit output: Feedback clock
		//.CLKFBOUTB(CLKFBOUT), // 1-bit output: Inverted CLKFBOUT
		// Status Ports: 1-bit (each) output: MMCM status ports
		.LOCKED(), // 1-bit output: LOCK
		// Clock Inputs: 1-bit (each) input: Clock input
		.CLKIN1(clk21), // 1-bit input: Clock
		// Control Ports: 1-bit (each) input: MMCM control ports
		.PWRDWN(), // 1-bit input: Power-down
		.RST(~lbus_rstn), // 1-bit input: Reset
		// Feedback Clocks: 1-bit (each) input: Clock feedback ports
		.CLKINSEL(1'b1),
		.CLKFBIN(CLKFBIN1) // 1-bit input: Feedback clock
		
	);
	// End of MMCME2_BASE_inst instantiation			












reg kvld, dvld, endvld, busy;


 aes_tiny aes_tiny( .clk(clk),  .rst(dvld),  .din(blk_din), .key(blk_kin), .dout(blk_dout),  .done(blk_dvld) );
		
	assign blk_kvld = kvld;
	assign blk_busy = busy;
	
	always @(posedge clk) begin
   if(lbus_rstn==0) begin
		dvld <=0;
		kvld <=0;
		busy  <=0;
	end
	else begin
		if(blk_krdy==1)
			kvld  <=1;
		else if(blk_krdy==0)
			kvld <=0;
			
		if(blk_drdy==1 && endvld==1) begin
			dvld <= 1;
			endvld <=0;
			busy  <=1;
		end
		else if(dvld==1)
			dvld<=0;
		else if(blk_dvld==1) begin
			endvld <=1;
			busy  <=0;			
		end
	end
	end






   reg enTX;
   wire TXdone,TXactive; 
	wire clkin1;						
	reg [7:0] data      [1023:0]; 							
   reg [9:0] addr;
   reg [31:0]count;
	reg [7:0] TXdata;
	wire[7:0] processedOut;
	reg en, transmit, transmitReg, counten;
	reg [2:0] bbyte;
	
wire [255:0] out;		
reg [255:0] outReg;		
parameter regsize =511;
	
uart_tx  uartOut(.i_Clock(clk1), .i_Tx_DV(transmitReg), .i_Tx_Byte(TXdata), .o_Tx_Active(TXactive), .o_Tx_Serial(tx), .o_Tx_Done(TXDone) );		
tdc_top tp (clk1, out);
tdc_decode tdc_decode(.clk(clk1), .rst(lbus_rstn), .chainvalue_i(outReg), .coded_o(processedOut)); 

integer index;
  always @(posedge clk1) begin
		if(lbus_rstn==0) begin
			addr <=0;
			transmit <=0;
			transmitReg <=0;
			counten <=0;
			bbyte <=0;
			for (index=0; index<1024; index=index+1) begin
         data[index] <= 0;
			en <=0;
         end
		end
		else if (dvld==1 && en==0 ) begin
			addr <=0;
			en   <=1;
			transmit <=0;
			transmitReg <=0;
			outReg <= out;
			
		end
		else if (en==1 && addr!=regsize && transmit ==0)  begin
         addr <= addr+1;
			//data[addr] <= addr[7:0];
			outReg <= out;
			data[addr] <= processedOut;
			transmit <=0;
			transmitReg <=0;
		end
		else if(en ==1 && addr==regsize && transmit ==0) begin
			en <=0;
			//transmit <=1;
			counten <=1;
			count  <=0;
			//transmitReg <=1;
			addr <=0;
			//TXdata <= data[addr];
			bbyte <= 0;
		end
		else if(counten==1) begin
			if(count==1024*1024*1) begin
				transmit <=1;
				transmitReg <=1;
				TXdata <= data[addr];
				
				counten <=0;
			end
			else
				count <= count +1;
		end		
		else if( transmit ==1) begin
				if(addr==regsize) begin
					transmit <=0;
					transmitReg <=0;
					addr <=0;
				end
				else if (transmitReg ==1) begin
						transmitReg <=0;
					end
				else if(TXactive==0) begin
					transmitReg <=1;
					TXdata <= data[addr];
					addr <= addr+1;
				end
				
		end 
	
  end

   IBUFG u10 (.I(lbus_clkn), .O(refclk)); 
//	BUFG u50 (.I(refclk), .O(clk0)); 
  //  assign clk0Out = clk0;


 wire CLKFBOUT, CLKFBIN;
 BUFG  u15 (.I(CLKFBOUT),   .O(CLKFBIN));
 
							
								
	MMCME2_ADV #(
		.BANDWIDTH("OPTIMIZED"), // Jitter programming (OPTIMIZED, HIGH, LOW)
		.CLKFBOUT_MULT_F(40.0), // Multiply value for all CLKOUT (2.000-64.000).
		.CLKFBOUT_PHASE(0.0), // Phase offset in degrees of CLKFB (-360.000-360.000).
		.CLKIN1_PERIOD(41), // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
		// CLKOUT0_DIVIDE - CLKOUT6_DIVIDE: Divide amount for each CLKOUT (1-128)
		.CLKOUT1_DIVIDE(10),
		.CLKOUT2_DIVIDE(5),
		.CLKOUT3_DIVIDE(1),
		.CLKOUT4_DIVIDE(1),
		.CLKOUT5_DIVIDE(1),
		.CLKOUT6_DIVIDE(1),
		.CLKOUT0_DIVIDE_F(40.0), // Divide amount for CLKOUT0 (1.000-128.000).
		// CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for each CLKOUT (0.01-0.99).
		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),
		.CLKOUT6_DUTY_CYCLE(0.5),
		// CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
		.CLKOUT0_PHASE(0.0),
		.CLKOUT1_PHASE(0.0),
		.CLKOUT2_PHASE(0.0),
		.CLKOUT3_PHASE(0.0),
		.CLKOUT4_PHASE(0.0),
		.CLKOUT5_PHASE(0.0),
		.CLKOUT6_PHASE(0.0),
		.CLKOUT4_CASCADE("FALSE"), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
		.DIVCLK_DIVIDE(1), // Master division value (1-106)
		.REF_JITTER1(0.0), // Reference input jitter in UI (0.000-0.999).
		.STARTUP_WAIT("FALSE") // Delays DONE until MMCM is locked (FALSE, TRUE)
	)
	MMCME2_BASE_inst (
		// Clock Outputs: 1-bit (each) output: User configurable clock outputs
		.CLKOUT0(clk0), // 1-bit output: CLKOUT0
		.CLKOUT1(clk1), // 1-bit output: CLKOUT1
		.CLKOUT2(clk3), // 1-bit output: CLKOUT2
		//.CLKOUT3(CLKOUT3), // 1-bit output: CLKOUT3
		//.CLKOUT4(CLKOUT4), // 1-bit output: CLKOUT4
		//.CLKOUT5(CLKOUT5), // 1-bit output: CLKOUT5
		//.CLKOUT6(CLKOUT6), // 1-bit output: CLKOUT6
		// Feedback Clocks: 1-bit (each) output: Clock feedback ports
		.CLKFBOUT(CLKFBOUT), // 1-bit output: Feedback clock
		//.CLKFBOUTB(CLKFBOUT), // 1-bit output: Inverted CLKFBOUT
		// Status Ports: 1-bit (each) output: MMCM status ports
		.LOCKED(LOCKED), // 1-bit output: LOCK
		// Clock Inputs: 1-bit (each) input: Clock input
		.CLKIN1(refclk), // 1-bit input: Clock
		// Control Ports: 1-bit (each) input: MMCM control ports
		.PWRDWN(PWRDWN), // 1-bit input: Power-down
		.RST(~lbus_rstn), // 1-bit input: Reset
		.CLKINSEL(1'b1),
		// Feedback Clocks: 1-bit (each) input: Clock feedback ports
		.CLKFBIN(CLKFBIN) // 1-bit input: Feedback clock
	);
	// End of MMCME2_BASE_inst instantiation				
			
								
endmodule // CHIP_SASEBO_GIII_AES


   
//================================================ MK_CLKRST
module MK_CLKRST (clkin, rstnin, clk, rst, clk00, clk11, clk22);
   //synthesis attribute keep_hierarchy of MK_CLKRST is no;
   
   //------------------------------------------------
   input  clkin, rstnin;
   output clk, rst;
   output clk00, clk11, clk22;
   //------------------------------------------------
   wire   refclk;
//   wire   clk_dcm, locked;
  
	//wire CLKFBOUT;


   //------------------------------------------------ reset
   MK_RST u20 (.locked(rstnin), .clk(clkin), .rst(rst));
endmodule // MK_CLKRST



//================================================ MK_RST
module MK_RST (locked, clk, rst);
   //synthesis attribute keep_hierarchy of MK_RST is no;
   
   //------------------------------------------------
   input  locked, clk;
   output rst;

   //------------------------------------------------
   reg [15:0] cnt;
   
   //------------------------------------------------
   always @(posedge clk or negedge locked) 
     if (~locked)    cnt <= 16'h0;
     else if (~&cnt) cnt <= cnt + 16'h1;

   assign rst = ~&cnt;
endmodule // MK_RST
