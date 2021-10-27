
# ---- get lidar data out of bag, sync front and rear lidar ----
python2 qolo/extract_lidar_from_rosbag.py

# ---- run detector, require Person-MinkUNet ----
python qolo/generate_detections.py

# ---- run tracker ----
export PYTHONPATH="${PYTHONPATH}:${PWD}/qolo/AB3DMOT"
echo $PYTHONPATH
python qolo/generate_tracks.py

# ---- plot scans, detections, and tracks ----
if [ ! -d "tmp" ]; then
    mkdir tmp
fi

python qolo/common.py

ffmpeg -y -r 15 -pattern_type glob -i "tmp/*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p "tmp/frames.mp4"