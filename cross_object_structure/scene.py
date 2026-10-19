"""Skip construction of an unused fixture when the frozen deployment has none.

The existing loader already omits the collider for height == 0. Its unconditional
footprint calculation can nonetheless reject a floor-mounted door. No positive
fixture, object geometry, dynamics or collision acceptance is changed here.
"""


def adapt_no_fixture(source,job):
    if job.get('fixed_fixture') is not None:return source
    old='fixture=fixture_box(manifest,asset_chain,asset_urdf,asset_rotation,asset_xyz,a.fixture_height_m)'
    if source.count(old)!=1:raise RuntimeError('NO_FIXTURE_SETUP_HOOK_CHANGED')
    new=old+" if a.fixture_height_m>0. else {'center':asset_xyz.tolist(),'size':[0.,0.,0.],'quaternion_wxyz':[1.,0.,0.,0.],'absent':True,'audit':{'reason':'frozen deployment requests no fixture; no collider is authored'}}"
    return source.replace(old,new)


def install_no_fixture_guard():
    import interactive_twin.plant as plant
    old=plant.adapt_loader_source
    def adapt(source,job):return adapt_no_fixture(old(source,job),job)
    plant.adapt_loader_source=adapt
