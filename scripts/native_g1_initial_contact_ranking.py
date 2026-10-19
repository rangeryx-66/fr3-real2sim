"""Shared ranking at initial contact, using existing cached closing geometry.

This score observes native closing-path accessibility; it changes neither
execution gates nor the frozen closure/controller/physical models.
"""
import numpy as np

def initial_contact_quality(quality):
    first=quality['first_contacts'];aperture=max((x['width_m'] for x in first.values()),default=.084)
    values={}
    for label in ['visual','native']:
        curve=sorted(quality['patch'][label]['samples'],key=lambda x:x['common_aperture_m'])
        values[label]=float(np.interp(aperture,[s['common_aperture_m'] for s in curve],[s['bilateral_score'] for s in curve]))
    score=min(values.values())/max(1.,1+quality['root_contact_count']) if quality['non_handle_closing_collision'] is None else 0.
    return {'first_native_contact_aperture_m':float(aperture),'bilateral_support_at_entry':values,'score':score,'ideal_final_support_score':quality['support_score'],'first_finger_apertures_m':{n:x['width_m'] for n,x in first.items()},'classification':'CLOSING_PATH_RANK_ONLY_NOT_PHYSICAL_GRASP_CERTIFICATION'}
