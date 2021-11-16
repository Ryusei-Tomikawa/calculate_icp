#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define SCAN_NUM 200

using namespace std;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  
  printf ("t = < x: =%6.3f, y:= %6.3f, z: =%6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char** argv){

    FILE *input_fp; //InputFile
    char *filename = "data.csv";

    //csv fileから格納する変数
    double scandata_1_x[SCAN_NUM];
    double scandata_1_y[SCAN_NUM];

    double scandata_2_x[SCAN_NUM];
    double scandata_2_y[SCAN_NUM];

    //点群の位置情報宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);


    //点群情報
    cloud_in->width    = SCAN_NUM;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);

    if(!(input_fp = fopen(filename, "r")))
    {
        cout << "Error: Read not data.csv ..." << std::endl;
        exit(1);
    }

    for (size_t i = 0; i < SCAN_NUM; i++)
    {
        fscanf(input_fp, "%lf,%lf,%lf,%lf", &scandata_1_x[i], &scandata_1_y[i], &scandata_2_x[i], &scandata_2_y[i]);
        /*printf("scandata_1_x = %lf", scandata_1_x[i]);
        cout << endl;
        printf("scandata_1_y = %lf", scandata_1_y[i]);
        cout << endl;
        printf("scandata_2_x = %lf", scandata_2_x[i]);
        cout << endl;
        printf("scandata_2_y = %lf", scandata_2_y[i]);
        cout << endl;
        */
    }
    
    //scandata_1
    for (size_t i = 0; i < cloud_in->points.size (); ++i)   {
        cloud_in->points[i].x = scandata_1_x[i];
        cloud_in->points[i].y = scandata_1_y[i];
        cloud_in->points[i].z = 0.0;

        std::cout << " スキャンデータ１:" <<
        cloud_in->points[i].x << " " <<
        cloud_in->points[i].y << " " <<
        std::endl;

        *cloud_out = *cloud_in;
    }

    //scandata_2
    for (size_t i = 0; i < cloud_in->points.size (); ++i){

        cloud_out->points[i].x = scandata_2_x[i];
        cloud_out->points[i].y = scandata_2_y[i];

        std::cout << " スキャンデータ2:" <<
        cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " <<
        std::endl;

    }

    //cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    //変換matrixを表示する
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);

    fclose(input_fp);

    return (0);
}
