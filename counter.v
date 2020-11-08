module Counter (count, clock); 
    input clock; 
    output count; 
    reg [3:0], count, count_reg; 

    initial count_reg = 0; 

    always @(posedge clock) 
        if(clock == 0)
            count <= 0;
        else
            count += 1;
    always @(posedge clock) 
        count = count_reg; 
endmodule