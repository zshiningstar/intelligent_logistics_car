when log corrimudata, we not only need the log command "log corrimudatasb ontime 0.01",
but also set the log rate param label such as <param name="log_corrimu_frequency" value=100/>
it is important that the frequency param you set must be corresponding to the log command. T = 0.01s -> f = 100HZ
