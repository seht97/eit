import pcl

class LandingDetector:
    def __init__(self):
        # Initialize
        pc = pcl.load('/home/magnus/EiT/pointcloud_pcd_files/210.576000000.pcd')

        #print(pc.size)

        #fil = pc.make_passthrough_filter()
        #fil.set_filter_field_name("z")
        #fil.set_filter_limits(0, 2.5)
        #cloud_filtered = fil.filter()
        #print(cloud_filtered.size)
        #pcl.save(cloud_filtered, '/home/magnus/EiT/filtered_pcd.pcd')
        seg = pc.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(0.03)
        indices, model = seg.segment()

        print(model)

        cloud_plane = pc.extract(indices, negative=False)
        print(cloud_plane.size)
        pcl.save(cloud_plane, '/home/magnus/EiT/segmented_pcd.pcd')
        
    def detect(self):
        # Run detection
        pass

def main():
    ld = LandingDetector()
    ld.detect()

if __name__ == "__main__":
    main()