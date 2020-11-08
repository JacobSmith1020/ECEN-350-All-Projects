module structural_2_to_1_mux(i0, i1, sel, out);
//declare variables
input i0, i1, sel;
output out;
wire out0, out1, out2;

//internal calculations
not INV1(out0, sel);
and AND1(out1, out0, i0);
and AND2(out2, sel, i1);

//calculation of output
or OR1(out, out1, out2);

endmodule