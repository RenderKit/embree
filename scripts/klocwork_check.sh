#!/bin/bash

## ======================================================================== ##
## Copyright 2009-2020 Intel Corporation                                    ##
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

set -e
export KW_BUILD_NUMBER=$(cat ./kw_build_number)
export KW_PROJECT_NAME=embree
export KW_CRITICAL_OUTPUT_PATH=./kw_critical.out

echo "Checking for critical issues in $KW_BUILD_NUMBER ..."
no_proxy=$KW_SERVER_IP curl -f --data "action=search&project=$KW_PROJECT_NAME&query=build:'$KW_BUILD_NUMBER'%20severity:Critical%20status:Analyze,Fix&user=$KW_USER&ltoken=$KW_LTOKEN" http://$KW_SERVER_IP:$KW_SERVER_PORT/review/api -o $KW_CRITICAL_OUTPUT_PATH
getCriticalCount() {
    cat $KW_CRITICAL_OUTPUT_PATH | wc -l
}
if [ -f $KW_CRITICAL_OUTPUT_PATH ]; then
    echo "****** ERROR ****** Critical issues found - $(getCriticalCount) in $KW_BUILD_NUMBER";
    cat $KW_CRITICAL_OUTPUT_PATH
    exit 1;
else
    echo "****** PASS ****** No critical issues were found in $KW_BUILD_NUMBER"
fi
