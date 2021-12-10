#!/bin/bash
# derived from https://stackoverflow.com/a/14203146/7961693
# sh eval_all.sh -e=py38cuda110 -t=0424_mds

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/crowdbot/miniconda3/bin/conda' 'shell.bash' 'hook' 2>/dev/null)"
if [ $? -eq 0 ]; then
  eval "$__conda_setup"
else
  if [ -f "/home/crowdbot/miniconda3/etc/profile.d/conda.sh" ]; then
    . "/home/crowdbot/miniconda3/etc/profile.d/conda.sh"
  else
    export PATH="/home/crowdbot/miniconda3/bin:$PATH"
  fi
fi
unset __conda_setup
# <<< conda initialize <<<

# database path
readonly ROSBAG_DATABASE=$(python3 parse_yaml.py ../data/data_path.yaml --get bagbase_dir)
readonly OUTPUT_DATABASE=$(python3 parse_yaml.py ../data/data_path.yaml --get outbase_dir)

for i in "$@"; do
  case $i in
  -e=* | --env=*)
    ENVIRONMENT="${i#*=}"
    shift # past argument=value
    ;;
  -t=* | --type=*)
    TYPE="${i#*=}"
    shift # past argument=value
    ;;
  *)
    # unknown option
    ;;
  esac
done
echo "CONDA ENVIRONMENT  = ${ENVIRONMENT}"
echo "CONTROL TYPE       = ${TYPE}"
# echo "SEARCH PATH        = ${SEARCHPATH}"
# echo "LIBRARY PATH       = ${LIBPATH}"
# echo "DEFAULT            = ${DEFAULT}"

conda activate ${ENVIRONMENT}
# conda info

echo "Number of rosbag in currnet control type:" $(ls -1 "${ROSBAG_DATABASE}/${TYPE}"/*".bag" | wc -l)

echo "########## Evaluate the performance ##########"
echo "##### eval_qolo_path.py #####"
python3 ../qolo/eval_qolo_path.py --overwrite -f ${TYPE}
echo "##### eval_qolo.py #####"
python3 ../qolo/eval_qolo.py --overwrite -f ${TYPE}
echo "##### eval_crowd.py #####"
python3 ../qolo/eval_crowd.py --overwrite -f ${TYPE}

echo "########## Finished!!! ##########"
