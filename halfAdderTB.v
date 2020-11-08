module structural_half_adder_test_bench();

reg a, b, cout, sum;

initial begin
    //there are four possible input combinations to test for a 2 input half adder
    a = 1'b0;
    b = 1'b0;
    structural_half_adder(a, b, cout, sum);
    $monitor("0 + 0 = ", sum, " with carry out of ", cout);

    a = 1'b0;
    b = 1'b1;
    structural_half_adder(a, b, cout, sum);
    $monitor("0 + 1 = ", sum, " with carry out of ", cout);

    a = 1'b1;
    b = 1'b0;
    structural_half_adder(a, b, cout, sum);
    $monitor("1 + 0 = ", sum, " with carry out of ", cout);

    a = 1'b1;
    b = 1'b1;
    structural_half_adder(a, b, cout, sum);
    $monitor("1 + 1 = ", sum, " with carry out of ", cout);

end

endmodule