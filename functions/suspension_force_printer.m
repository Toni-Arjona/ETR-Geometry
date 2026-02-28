function suspension_force_printer(FORCE_VECTOR)
    fprintf(" --- SUSPENSION FORCE [N] PRINTER ---\n");
    fprintf("01: LWF support -----------"); FORCE_VECTOR(1).print();
    fprintf("02: LWR support -----------"); FORCE_VECTOR(2).print();
    fprintf("03: LW  knuckle joint -----"); FORCE_VECTOR(3).print();
    fprintf("04: UWF support -----------"); FORCE_VECTOR(4).print();
    fprintf("05: UWR support -----------"); FORCE_VECTOR(5).print();
    fprintf("06: UW  knuckle joint -----"); FORCE_VECTOR(6).print();
    fprintf("07: PR  wishbone ----------"); FORCE_VECTOR(7).print();
    fprintf("08: PR  rocker joint ------"); FORCE_VECTOR(8).print();
    %fprintf("09: -----------------------"); FORCE_VECTOR(9).print();
    %fprintf("10: -----------------------"); FORCE_VECTOR(10).print();
    fprintf("11: TR  knuckle joint -----"); FORCE_VECTOR(11).print();
    fprintf("12: TR  axis joint --------"); FORCE_VECTOR(12).print();
    %fprintf("13: -----------------------"); FORCE_VECTOR(13).print();
    %fprintf("14: -----------------------"); FORCE_VECTOR(14).print();
    %fprintf("15: -----------------------"); FORCE_VECTOR(15).print();
    fprintf("16: DP  body joint --------"); FORCE_VECTOR(16).print();
    fprintf("17: DP  rocker joint ------"); FORCE_VECTOR(17).print();
    fprintf("18: W   spindle -----------"); FORCE_VECTOR(18).print();
    fprintf("19: W   centre ------------"); FORCE_VECTOR(19).print();
    fprintf("20: R   axis first joint --"); FORCE_VECTOR(20).print();
    fprintf("21: R   axis second joint -"); FORCE_VECTOR(21).print();


end

