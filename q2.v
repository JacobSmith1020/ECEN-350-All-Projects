module q2;

reg clock;

initial begin
    clock = 0;
    #20 clock = 1; //set first rising edge to happen 20 units after program starts
    forever begin
        #4 clock = !clock; //every four seconds, reverse the signal of the clock, switching from high to low and low to high
    end
end 

endmodule