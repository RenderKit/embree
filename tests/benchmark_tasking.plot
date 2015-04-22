## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

set style line 101 lc rgb "green" lt 1
set style line 102 lc rgb "green" lt 1
set style line 103 lc rgb "#008800" lt 1
set style line 104 lc rgb "grey" lt 1
set style line 105 lc rgb "yellow" lt 1
set style line 106 lc rgb "orange" lt 1
set style line 107 lc rgb "red" lt 1
set style line 108 lc rgb "blue" lt 1
set style line 109 lc rgb "#000088" lt 1

#set term postscript enhanced color 
set term png

set xtics rotate 90
set xlabel 'Items'
set logscale x
set xrange [1000:8000000]

set yrange [0:5000]
set ylabel 'Performance in M/s'

#set output "benchmark_reduce.png"
#set title "benchmark_reduce Single Socket HSW"
#plot 'benchmark_reduce_sequential.csv' using ($1):($6) t "single threaded" with lines ls 108, \
#     'benchmark_reduce_lockstep.csv' using ($1):($6) t "lockstep taskscheduler" with lines ls 102, \
#     'benchmark_reduce_tbb.csv' using ($1):($6) t "threading building blocks" with lines ls 107, \
#     'benchmark_reduce_mytbb.csv' using ($1):($6) t "threading building blocks" with lines ls 108

#set output "benchmark_sort.png"
#set title "benchmark_sort Single Socket HSW"
#plot 'benchmark_sort_sequential.csv' using ($1):($6) t "single threaded" with lines ls 108, \
#     'benchmark_sort_lockstep.csv' using ($1):($6) t "lockstep taskscheduler" with lines ls 102, \
#     'benchmark_sort_tbb.csv' using ($1):($6) t "threading building blocks" with lines ls 107

set output "benchmark_box_reduce.png"
set title "benchmark_box_reduce KNC"
plot 'benchmark_reduce_lockstep.csv' using ($1):($6) t "lockstep taskscheduler" with lines ls 102, \
     'benchmark_reduce_tbb.csv' using ($1):($6) t "threading building blocks" with lines ls 107, \
     'benchmark_reduce_tbb_new.csv' using ($1):($6) t "threading building blocks (spin)" with lines ls 108


