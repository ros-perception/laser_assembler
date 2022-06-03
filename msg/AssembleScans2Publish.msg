# The time interval on which we want to aggregate scans
time begin
# The end of the interval on which we want to assemble scans or clouds
time end
# How long to wait, in wall clock time, from when the assembly request is
# received to receive at least one scan that has a stamp >= end prior to
# publishing the cloud. This can be used to wait some amount of time (up to the
# blocking_period) to fill the full request. This is helpful to ensure cloud
# stamps and contents match between multiple assemblers, for instance.
duration blocking_period
