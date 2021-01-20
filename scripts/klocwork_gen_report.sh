#!/bin/bash -xe
## Copyright 2020-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

KW_SERVER_API_URL=http://$KW_SERVER_IP:$KW_SERVER_PORT/review/api
KW_BUILD_NAME=$(cat $CI_PROJECT_DIR/klocwork/build_name)
KW_BUILD_LOG_FILE=$CI_PROJECT_DIR/klocwork/build.log

export PATH="$SHARED_TOOLS_PATH:$PATH"

[ -f $KW_BUILD_LOG_FILE ] || (echo "Build log file not found. Expected to be in: $KW_BUILD_LOG_FILE." ; exit 1;)

mkdir -p $CI_PROJECT_DIR/klocwork
report_file=$CI_PROJECT_DIR/klocwork/report.log
echo "------------------" >> $report_file
echo "Report generated at: "$(date '+%d/%m/%Y %H:%M:%S') >> $report_file
echo "Project source code url: $CI_PROJECT_URL" >> $report_file
echo "Project source code sha: $CI_COMMIT_SHA" >> $report_file
echo "Klocwork server: http://$KW_SERVER_IP:$KW_SERVER_PORT" >> $report_file
echo "------------------" >> $report_file

echo -e "\n\n\n" >> $report_file

# Get all issues list and put to report file
column_list=".id, .code, .severity, .state, .status, .taxonomyName, .owner, .url, .file, .line"
echo "------------------" >> $report_file
echo "Issues list:" >> $report_file
echo "------------------" >> $report_file
echo $column_list | sed 's/\\t/ ,/g' | column -t -s, >> $report_file
echo "------------------" >> $report_file
curl -f --data "action=search&project=$KW_PROJECT_NAME&query=build:'$KW_BUILD_NAME'&user=$KW_USER&ltoken=$KW_LTOKEN" $KW_SERVER_API_URL | jq-linux64 "[${column_list}] | @tsv" | sed 's/\\t/|/g' | column -t -s'|' | cut -d'"' -f2 >> $report_file

echo -e "\n\n\n" >> $report_file

# Attach build log to report file
echo "------------------" >> $report_file
echo "Build & scan log:" >> $report_file
echo "------------------" >> $report_file
cat $KW_BUILD_LOG_FILE >> $report_file
