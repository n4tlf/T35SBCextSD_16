/************************************************************************
*   FILE:   ParShiftReg.v      Ver 0.2      Jan. 30, 2023               *
*                                                                       *
*   This function is used to create wait delays for I/O or ROM access   *
*       It is similar to a 74165 TTL chip or BDF emulation.             *
*       load is used to load the parallel input into the temp register  *
*       clr is disabled for now, Active LOW will set temp HIGH, no delay*
*       SerIn is the LAST bit sent out.  It also sets the post-delay    *
*           logic level, so for now, keep SerIn active HIGH             *
*       ParIn is the first eight bits used to create a wait delay output*
*           Input: FF = no wait states, 00 = 312ns (max wait)           *
*           To add wait states, start with MOST significant bit first   *
*           (ie: 7F=1 wait) 3F = 2 wait states, etc.                    *
*           DO NOT START WITH LSB first,as wait state                   *
*   TFOX, N4TLF January 20, 2023   You are free to use it               *
*       however you like.  No warranty expressed or implied             *
************************************************************************/

module ParShiftReg
    (
    input           clk,    // shift on Positive clock edge of cpuClock
//    input           clr,     // active low to clear    DISABLED for now
    input           SerIn,  // shift register serial input 
    input   [7:0]   ParIn,  // Shift register parallel input
    input           load,   // Shift (low)/Load (high) input
    output          qout);  // single bit ready/wait output
   
reg [8:0]   temp;

always @(posedge(clk))  begin    //or negedge(clr))  begin
//    if(clr == 1'b0)             // if clear was sent
//        temp <= 9'b1;           // clear the temp reg
//    else
        if(load) begin                  // if loading the shift register
            temp[8:1] <= ~ParIn[7:0];      // set the parallel data
            temp[0] <= !SerIn;
            end
        else begin                          // otherwise shift the register
            temp = temp << 1;               //SerIn};
        end
    end
    
assign qout = !temp[8];

endmodule

