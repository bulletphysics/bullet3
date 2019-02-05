import numpy as np

def compute_return(rewards, gamma, td_lambda, val_t):
    # computes td-lambda return of path
    path_len = len(rewards)
    assert len(val_t) == path_len + 1

    return_t = np.zeros(path_len)
    last_val = rewards[-1] + gamma * val_t[-1]
    return_t[-1] = last_val

    for i in reversed(range(0, path_len - 1)):
        curr_r = rewards[i]
        next_ret = return_t[i + 1]
        curr_val = curr_r + gamma * ((1.0 - td_lambda) * val_t[i + 1] + td_lambda * next_ret)
        return_t[i] = curr_val
    
    return return_t