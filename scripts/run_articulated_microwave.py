"""Small PhysX microwave with a robot mounted outside the front table edge."""
import sys
from run_articulated_47686 import main

if __name__=='__main__':
    sys.argv=[sys.argv[0],
        '--asset-root','/data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1',
        '--task','抓住微波炉把手打开门','--target-part','microwave door handle',
        '--sam3-prompt','microwave door handle','--sam3-center-crop-size','0',
        '--asset-x','.50','--asset-y','-.05','--asset-yaw-deg','-90',
        '--fixture-height-m','.04','--base-pose','.50','-.55','-.10','150',
        '--support-bottom-z','-.76',
        '--home-q','0','.5','.5','-1.3','0','0','0',
        '--output','results/articulated_microwave_7320',*sys.argv[1:]]
    main()
