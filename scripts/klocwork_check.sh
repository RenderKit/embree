#!/bin/bash -xe
## Copyright 2020-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

KW_ISSUES_FILE=/tmp/issues
KW_SERVER_API_URL=http://$KW_SERVER_IP:$KW_SERVER_PORT/review/api
KW_BUILD_NAME=$(cat $CI_PROJECT_DIR/klocwork/build_name)

echo "Checking for issues in $KW_BUILD_NAME ..."
curl -f --data "action=search&project=$KW_PROJECT_NAME&query=build:'$KW_BUILD_NAME'%20status:Analyze,Fix,Fix%20in%20Next%20Release,Fix%20in%20Later%20Release,Defer,Filter&user=$KW_USER&ltoken=$KW_LTOKEN" $KW_SERVER_API_URL -o $KW_ISSUES_FILE
getCriticalCount() {
    cat $KW_ISSUES_FILE | wc -l
}
if [ -f $KW_ISSUES_FILE ]; then
    echo "Issues found - $(getCriticalCount) in $KW_BUILD_NAME";
    while IFS= read -r line; do echo $line | python -m json.tool; done < $KW_ISSUES_FILE
    exit 1;
else
    echo "There are no issues which should be take care in $KW_BUILD_NAME"
fi

