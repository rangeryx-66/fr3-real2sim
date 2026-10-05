"""Import this adapter inside the official Ditto inference environment."""
def load_ditto_sample(path,device='cuda'):
    import numpy as np
    import torch
    with np.load(path) as data:
        return {key:torch.from_numpy(data[key]).float().to(device) for key in ('pc_start','pc_end')}
