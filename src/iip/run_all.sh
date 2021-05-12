trap "exit" INT

mkdir -p "iip_log"

LIMIT=107
for idx in $(seq 0 ${LIMIT})
do
    FILE_IDX=$(printf %010d $(($idx)))
    FILE="../data/global/${FILE_IDX}.ply"
    if [ -f ${FILE} ]
    then
        echo "${FILE} already exists."
        continue
    else
        echo "Generate global coordinates..."
        mkdir -p "../data/global"
        /usr/bin/time -v -o iip_log/time_oxt2pose.out python iip/oxt2pose/main.py > iip_log/info_oxt2pose.log
        cat iip_log/time_oxt2pose.out | grep "Elapsed"
        echo "Done." 
    fi
done

echo "Get innovations and register point clouds..."
mkdir -p "iip/pose2reg/build"
cd "iip/pose2reg/build"
cmake ..
make
mkdir -p "../../../../data/innovation"
mkdir -p "../../../../data/reference"
/usr/bin/time -v -o ../../../iip_log/time_pose2reg.out ./pc_encoder > ../../../iip_log/info_pose2reg.log
cat ../../../iip_log/time_pose2reg.out | grep "Elapsed"
echo "Done."

echo "Finished IIP registration."
