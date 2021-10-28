import pcl
import numpy as np

# Drone size
DRONE_WIDTH = 0.7
DRONE_HEIGHT = 0.65

class LandingDetector:
    def __init__(self):
        # Initialize
        self.pc = pcl.load('/home/magnus/EiT/pointcloud_pcd_files/210.576000000.pcd')

    def add_noise(self):
        pc = np.asarray(self.pc)
        #print(pc)
        pc[:,2] += 0.1*np.random.rand(pc.shape[0])
        #print(pc)
        self.pc = pcl.PointCloud(pc)
        pcl.save(self.pc, '/home/magnus/EiT/noisy_pcd.pcd')
        
    def detect(self):
        # Run detection
        fil = self.pc.make_passthrough_filter()
        fil.set_filter_field_name("x")
        fil.set_filter_limits(-DRONE_WIDTH/2, DRONE_WIDTH/2)
        cloud_filtered = fil.filter()
        fil = cloud_filtered.make_passthrough_filter()
        fil.set_filter_field_name("y")
        fil.set_filter_limits(-DRONE_HEIGHT/2, DRONE_HEIGHT/2)
        cloud_filtered = fil.filter()
        pcl.save(cloud_filtered, '/home/magnus/EiT/filtered_pcd.pcd')
        
        # Smoothing
        #fil = self.pc.make_statistical_outlier_filter()
        #fil.set_mean_k(50)
        #fil.set_std_dev_mul_thresh(1.0)
        #cloud_filtered = fil.filter()
        #pcl.save(cloud_filtered, '/home/magnus/EiT/filtered_pcd.pcd')

        seg = cloud_filtered.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(0.2)
        indices, model = seg.segment()

        cloud_plane = cloud_filtered.extract(indices, negative=False)
        pcl.save(cloud_plane, '/home/magnus/EiT/segmented_pcd.pcd')
        print(cloud_plane.size)
        print(cloud_filtered.size)
        if cloud_plane.size == cloud_filtered.size:
            print("True")
        else:
            print("False")

def main():
    ld = LandingDetector()
    ld.add_noise()
    ld.detect()

if __name__ == "__main__":
    main()