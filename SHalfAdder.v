module structural_half_adder(a, b, cout, sum);
//declare variables
input a, b; 
output cout, sum; 
wire sum1, sum2, sum3; 

//internal calculations
nand NAND1(sum3, a, b); 
nand NAND2(sum1, a, sum3); 
nand NAND3(sum2, sum3, b); 

//calculation of sum and cout
nand NAND4(sum, sum1, sum2); 
assign cout = sum3; 
 
endmodule