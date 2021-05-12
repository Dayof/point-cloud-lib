trap "exit" INT

mkdir -p "icp_log"
mkdir -p "../data/icp_global"

LIMIT=108
for idx in $(seq 1 ${LIMIT})
do
    FILE_IDX=$(printf %010d $(($idx-1)))
    FILE="../data/icp_global/${FILE_IDX}.ply"
    if [ -f ${FILE} ]
    then
        echo "${FILE} already exists."
        continue
    fi

    echo "Iteration ${idx}:"
    /usr/bin/time -v -o icp_log/time_${idx}.out python icp/global.py -n ${idx} > icp_log/info_${idx}.log
    cat icp_log/time_${idx}.out | grep "Elapsed"
    echo "Done."
done
echo "Finished ICP registration with ${LIMIT} point clouds."
