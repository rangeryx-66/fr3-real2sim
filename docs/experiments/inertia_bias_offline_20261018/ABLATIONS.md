# Diagnostic ablations

All alternatives are offline diagnostics. Common 211-sample boundaries exclude the complete maximum cascade; original reproduction retains its frozen 121-sample boundaries. Raw-target CV means the **original compensated** torque (itself biased), not unprocessed sensor truth. No candidate is promoted to the runtime. Validation never enters fitting or selection.

| Variant | I kg m² | b | C Nm | FIT RMSE mNm | FIT LOSO mNm | Original-target LOSO mNm | Held-out torque mNm | Held-out angle, original input (deg) | Full 9.175s angle (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| original | 0.165298 | 0.302645 | 0.0177257 | 3.05894 | 3.06369 | 3.06369 | 1.16059 | 0.0578552 | 0.122532 |
| baseline_common_edges | 0.166824 | 0.291247 | 0.0179319 | 3.04484 | 3.04698 | 3.04698 | 1.04939 | 0.0493035 | 0.16155 |
| derivative_position_SG2_61 | 0.165138 | 0.290846 | 0.0179473 | 1.97048 | 2.01471 | 3.05555 | 0.837486 | 0.0544704 | 0.165244 |
| derivative_native_velocity_SG1_61 | 0.158213 | 0.288075 | 0.0179875 | 1.40921 | 1.48444 | 3.05449 | 0.623732 | 0.0530438 | 0.145424 |
| derivative_position_gradient_SG0_61 | 0.157162 | 0.291916 | 0.0179309 | 1.55879 | 1.62476 | 3.05465 | 0.730391 | 0.0539134 | 0.14173 |
| torque_SG0_61 | 0.159791 | 0.291585 | 0.0179345 | 1.483 | 1.51327 | 3.05078 | 0.46211 | 0.0531893 | 0.148221 |
| torque_SG2_equivalent_61 | 0.143948 | 0.28768 | 0.0179776 | 0.86772 | 0.887001 | 3.07797 | 0.292879 | 0.0419723 | 0.0875122 |
| joint_tool_window_61 | 0.137652 | 0.292821 | 0.0179057 | 1.33679 | 1.35895 | 3.09657 | 0.717595 | 0.0459284 | 0.072778 |
| matched_all_61 | 0.145232 | 0.288145 | 0.0179752 | 0.875981 | 0.897903 | 3.07401 | 0.295251 | 0.0450781 | 0.0954683 |
| single_pass_61 | 0.112231 | 0.291046 | 0.0179212 | 0.158053 | 0.164548 | 3.26165 | 0.080454 | 0.0400172 | 0.0472026 |
| derivative_position_SG2_121 | 0.166824 | 0.291247 | 0.0179319 | 3.04484 | 3.04698 | 3.04698 | 1.04939 | 0.0493035 | 0.16155 |
| derivative_native_velocity_SG1_121 | 0.158323 | 0.287819 | 0.0179939 | 2.35159 | 2.36467 | 3.05088 | 1.01904 | 0.054648 | 0.148163 |
| derivative_position_gradient_SG0_121 | 0.152941 | 0.291059 | 0.0179434 | 2.75983 | 2.76168 | 3.05868 | 1.05434 | 0.0535086 | 0.129 |
| torque_SG0_121 | 0.15261 | 0.29061 | 0.01792 | 1.22941 | 1.23537 | 3.05815 | 0.343417 | 0.0354386 | 0.100962 |
| torque_SG2_equivalent_121 | 0.129924 | 0.276311 | 0.018121 | 0.466166 | 0.468534 | 3.144 | 0.143105 | 0.0214946 | 0.0255063 |
| joint_tool_window_121 | 0.166824 | 0.291247 | 0.0179319 | 3.04484 | 3.04698 | 3.04698 | 1.04939 | 0.0493035 | 0.16155 |
| matched_all_121 | 0.130925 | 0.287774 | 0.0179663 | 0.473343 | 0.478459 | 3.13613 | 0.159711 | 0.034617 | 0.0416285 |
| single_pass_121 | 0.112231 | 0.288449 | 0.0179574 | 0.0860958 | 0.0943968 | 3.2612 | 0.0680672 | 0.0374867 | 0.0481189 |
| derivative_position_SG2_181 | 0.160073 | 0.2933 | 0.0178831 | 3.67094 | 3.67216 | 3.0504 | 1.03037 | 0.0390088 | 0.125514 |
| derivative_native_velocity_SG1_181 | 0.151066 | 0.288216 | 0.0179803 | 3.20179 | 3.20242 | 3.06505 | 1.03084 | 0.0494706 | 0.119025 |
| derivative_position_gradient_SG0_181 | 0.138789 | 0.291535 | 0.0179199 | 3.60955 | 3.61396 | 3.11773 | 1.05973 | 0.0422074 | 0.0715381 |
| torque_SG0_181 | 0.14484 | 0.289546 | 0.0179241 | 0.949337 | 0.95199 | 3.07916 | 0.259847 | 0.0275014 | 0.0666668 |
| torque_SG2_equivalent_181 | 0.122104 | 0.263391 | 0.0182969 | 0.270897 | 0.281188 | 3.19729 | 0.0758494 | 0.0128041 | 0.0383935 |
| joint_tool_window_181 | 0.178842 | 0.293512 | 0.0178757 | 4.27033 | 4.2712 | 3.05481 | 1.09033 | 0.0425946 | 0.176111 |
| matched_all_181 | 0.12297 | 0.287379 | 0.0179683 | 0.236144 | 0.240205 | 3.18384 | 0.09982 | 0.0317914 | 0.0291098 |
| single_pass_181 | 0.112231 | 0.287004 | 0.0179773 | 0.0484858 | 0.0592744 | 3.2611 | 0.061268 | 0.0358262 | 0.04885 |
| FT_shift_-8 | 0.146716 | 0.357612 | 0.0170144 | 2.94303 | 2.95988 | 3.11153 | 1.0112 | 0.102603 | 0.150046 |
| FT_shift_-4 | 0.157977 | 0.325743 | 0.017455 | 2.97951 | 2.98631 | 3.06188 | 1.15475 | 0.0766781 | 0.158536 |
| FT_shift_-2 | 0.163167 | 0.308168 | 0.0176982 | 2.99317 | 2.99656 | 3.04964 | 1.05103 | 0.0622968 | 0.161574 |
| FT_shift_-1 | 0.165002 | 0.299487 | 0.0178239 | 3.0175 | 3.02008 | 3.04737 | 1.04484 | 0.0588516 | 0.16639 |
| FT_shift_+0 | 0.166824 | 0.291247 | 0.0179319 | 3.04484 | 3.04698 | 3.04698 | 1.04939 | 0.0493035 | 0.16155 |
| FT_shift_+1 | 0.168197 | 0.28308 | 0.0180522 | 3.09408 | 3.09704 | 3.04883 | 1.06383 | 0.0478575 | 0.167449 |
| FT_shift_+2 | 0.169057 | 0.272661 | 0.0181917 | 3.10278 | 3.10712 | 3.05255 | 1.02551 | 0.0406262 | 0.159926 |
| FT_shift_+4 | 0.170752 | 0.254634 | 0.0184395 | 3.15288 | 3.1606 | 3.06408 | 0.936514 | 0.0416268 | 0.153394 |
| FT_shift_+8 | 0.175097 | 0.221342 | 0.0188971 | 3.23108 | 3.24171 | 3.09821 | 0.857529 | 0.0803023 | 0.147402 |
| centered_tool_angular_velocity | 0.166821 | 0.291259 | 0.0179317 | 3.04481 | 3.04694 | 3.04698 | 1.04938 | 0.0493112 | 0.161548 |
| weak_0.5 | 0.111237 | 0.291454 | 0.0179114 | 0.135724 | 0.14301 | 3.26963 | 0.0816846 | 0.0374152 | 0.0514177 |
| weak_1.0 | 0.111322 | 0.288356 | 0.0179527 | 0.0459902 | 0.0575296 | 3.2693 | 0.0640617 | 0.0328959 | 0.0536205 |

Torque errors in different filtered/weak spaces are not directly comparable as sensor-noise estimates. Forward comparisons above always use the same original compensated torque and sole native measured initial velocity; no subsequent state correction. The complete segment includes derivative-edge extrapolation and is explicitly a boundary-sensitivity result.

Ordinary full-fit residual/acceleration correlation is near zero by least-squares construction. FIT leave-segment-out correlations and fixed-X torque-change projections are in DIAGNOSTICS.json; neither a zero fitted correlation nor lower filtered RMSE alone establishes physical accuracy.
