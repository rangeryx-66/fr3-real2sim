# Reactive grip selection

The strong reactive baseline chooses the greatest demonstrated rail
transmission capacity rather than assuming the largest pinch request is
always most usable. Measured normal load breaks capacity ties and supplies
the fallback when all empirical capacity bounds are zero.

The fastest common qualified reference and original live feedback remain
available. This ranking is fixed before changed-condition comparisons. It
does not read mass/damping regression or hidden plant parameters. No
experimental calibration values are included in this algorithm note.
