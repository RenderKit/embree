#!/bin/bash
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

files=$1
jq_tool="$SHARED_TOOLS_PATH/jq-linux64"
logfile="bdba.log"
failed=0
for file_path in $files ; do
    echo -e "\n" >> $logfile
    #upload file
    upload_response=`curl -k -H "Authorization: Bearer $BDBA_TOKEN" -H "Group: $BDBA_GROUP" -T $file_path "$BDBA_SERVER/api/upload/"`
    product_id=`echo "$upload_response" | $jq_tool -r '.results.product_id'`
    if [ $product_id == "null" ]; then
        echo "Cannot upload file $file_path" >> $logfile
        failed=1
        continue
    fi
    report_url=`echo "$upload_response" | $jq_tool -r '.results.report_url'`

    echo "Scan upload of $file_path completed - product id: $product_id ($report_url)" >> $logfile

    set +e
    MAX_RETRY=600

    RETRY_COUNTER="0"
    while [ $RETRY_COUNTER -lt $MAX_RETRY ]; do
        response=`curl -s -X GET -H "Authorization: Bearer $BDBA_TOKEN" -k $BDBA_SERVER/api/product/$product_id/`
        CMD_RETURN_CODE=$?

        status=`echo "$response" | $jq_tool -r '.results.status'`
        verdict=`echo "$response" | $jq_tool -r '.results.summary.verdict.short'`
        if [ $CMD_RETURN_CODE == 0 ] && [[ $status == "R" ]]; then
            echo $response | python -m json.tool
            echo "Verdict: $verdict" >> $logfile
            if [ $verdict != "Pass" ] && [ $verdict != "null" ]; then
                echo "There is a problem - please check report $report_url" >> $logfile
                failed=1
            fi
            # Download pdf report & components list
            file_name=`basename "$file_path"`
            echo "File name: $file_name"
            curl -H "Authorization: Bearer $BDBA_TOKEN" -k $BDBA_SERVER/api/product/$product_id/pdf-report?cvss_version=3 -o ${file_name}_report.pdf
            curl -H "Authorization: Bearer $BDBA_TOKEN" -k $BDBA_SERVER/api/product/$product_id/csv-libs -o ${file_name}_components.csv
            break
        fi
        RETRY_COUNTER=$[$RETRY_COUNTER+1]
        echo "Scan not finished yet, [$RETRY_COUNTER/$MAX_RETRY] - checking again ... "
        sleep 20
    done

    set -e
    if [ $RETRY_COUNTER -ge $MAX_RETRY ]; then
        failed=62
        continue
    fi
done

cat $logfile

exit $failed
