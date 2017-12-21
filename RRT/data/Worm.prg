ProgramFile
! Planner: RRT Connect (default)
! Settings:
!    er:syncipophases
!    er:unload
!    mp:rrtconnect
!    mv:erdiscrete = 0.01,1.0
!    ps:collapseclosevertices
!    ps:reducevertices
!    ps:shortcutpath
!    ps:simplifymax
!    rrtconnect:setrange = 0.1
! Time: 9.772566s
! Collision Checks: 190454
! Path Length: 7.288149
! Path Costs: 0.000000
EndInit
ERC CURRENT_DEVICE SET WORM
ERC DISPLAY_CROBOT ON
SPEED_PTP_OV	80.0000
SPEED_CP_OV	80.0000
SPEED_ORI_OV	80.0000
ACCEL_PTP_OV	100.0000
ACCEL_CP_OV	100.0000
ACCEL_ORI_OV	100.0000
OV_PRO	100.0000
ERC NO_DECEL ON
ZONE	0.0000
call Path()
EndProgramFile

Fct Path()
JUMP_TO_AX        0.5000        0.4500        0.0000      -90.0000       -0.0000
PTP_AX        0.3726        0.4518       31.0512      -64.0722        7.3564
PTP_AX        0.3930        0.4302       34.6355      -54.9239       11.5815
PTP_AX        0.4669        0.4188       36.4057      -41.2205       17.2582
PTP_AX        0.5249        0.4103       35.9552      -26.5991       26.5030
PTP_AX        0.5330        0.4098       33.0723      -18.6242       35.1389
PTP_AX        0.6094        0.4137       32.7634        0.8286       61.8449
PTP_AX        0.6721        0.4424       50.1144       17.7483       36.7468
PTP_AX        0.6867        0.4496       54.5163       21.7137       29.9448
PTP_AX        0.7078        0.4772       65.6555       21.1508      -10.9648
PTP_AX        0.7202        0.5169       69.5098       14.4565      -39.0764
PTP_AX        0.7380        0.5952       69.8628      -20.6820      -15.6673
PTP_AX        0.7750        0.6500       90.0000      -90.0000       90.0000
EndFct
