# load_metrics = (
#     'crowd_density5',
#     'crowd_density10',
#     'min_dist',
#     'avg_crowd_density5',
#     'avg_crowd_density10',
#     'avg_min_dist',
# )

metrics = ('avg_crowd_density5', 'avg_crowd_density10', 'avg_min_dist')

frames = []

for eval_dir in eval_dirs:

    print("Reading results from {}".format(eval_dir))

    # new a CrowdBotDatabase() instance
    eval_database = CrowdBotDatabase(folder=eval_dir)

    m_dict = {'seq': eval_database.seqs}
    m_dict.update(
        {'control_type': [eval_dir[5:] for i in range(eval_database.nr_seqs())]}
    )
    m_dict.update({k: [] for k in metrics})

    for idx, seq in enumerate(eval_database.seqs):
        eval_res_dir = os.path.join(eval_database.metrics_dir)
        crowd_eval_npy = os.path.join(eval_res_dir, seq + "_crowd_eval.npy")
        crowd_eval_dict = np.load(
            crowd_eval_npy,
            allow_pickle=True,
        ).item()

        for iidx, val in enumerate(metrics):
            m_dict[metrics[iidx]].append(crowd_eval_dict[val])

    dir_df = pd.DataFrame(m_dict)
    dir_df.columns = ['seq', 'control_type'] + list(metrics)

    frames.append(dir_df)

crowd_metrics_df = pd.concat(frames)

# crowd__metrics_df.head()
print(crowd_metrics_df.to_string(index=False))
