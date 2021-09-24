function chi_dot = bank_to_turn(chi,psi,phi)
g= 9.81;
V_g = 580;
chi_dot = (g/V_g)*tan(phi)*cos(chi-psi);
end 