#pragma once
#define CGAL_EIGEN3_ENABLED
#define PCL_ALWAYS
#define BOOST_SP_USE_QUICK_ALLOCATOR
#define BOOST_SP_
#define PCL_EIGEN_SIZE_MIN_PREFER_DYNAMIC
#define PCL_BOOST_

// header filesystem
#include<boost/shared_ptr.hpp>
#include<iostream>
#include<filesystem>
#include<vector>
#include<fstream>
#include<utility>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/point_representation.h>
#include<pcl/io/io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/voxel_grid_label.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/pcl_base.h>
#include<pcl/pcl_config.h>
#include<pcl/pcl_exports.h>
#include<pcl/exceptions.h>
#include<pcl/common/common.h>
#include<pcl/common/io.h>
#include<pcl/common/colors.h>
#include<pcl/pcl_macros.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/ply/ply.h>
#include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply/io_operators.h>
#include<pcl/PCLPointField.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid_label.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_handlers.h>
#include<pcl/visualization/boost.h>
#include<pcl/visualization/point_cloud_color_handlers.h>
#include<pcl/visualization/boost.h>
#include<pcl/visualization/eigen.h>
#include<pcl/visualization/vtk.h>

// CGAL header
#include<CGAL/basic.h>
#include<CGAL/basic_classes.h>
#include<CGAL/kernel_basic.h>
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/grid_simplify_point_set.h>
#include<CGAL/IO/read_xyz_points.h>
#include<CGAL/property_map.h>
#include<CGAL/Polyhedron_3.h>
#include<CGAL/convex_hull_3.h>
#include<CGAL/Memory_sizer.h>
#include<CGAL/IO/write_ply_points.h>
#include<CGAL/Simple_cartesian.h>
#include<CGAL/Subdivision_method_3.h>

using namespace pcl;
using namespace CGAL;
using namespace boost;

typedef pcl::PointCloud < pcl::PointXYZ >  pcl_point;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3< K >  Polyhedron;

typedef K::Point_3 Point;
typedef K::Vector_3 Vector;

int main(int argc, const char** argv) {
	try
	{
		// create pcd objects
		PointCloud<PointXYZRGB> ::Ptr src(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZ> ::Ptr dest(new PointCloud<PointXYZ>);
		PointCloud<PointXYZRGB> ::Ptr outcloud(new PointCloud<PointXYZRGB>);

		// load pcd data
		pcl::PLYReader reader;
		reader.read<pcl::PointXYZRGB>("src.ply", *src);
		// downsampled
		VoxelGrid<PointXYZRGB> passvox;
		passvox.setInputCloud(src);
		passvox.setLeafSize(0.01f, 0.01f, 0.01f);
		passvox.filter(*outcloud);

		//PointXYZRGB to PointXYZ
		pcl::copyPointCloud(*outcloud, *dest);

		//declare cgal vector of point
		std::vector<Point> points;
		pcl_point::iterator start_id = dest->begin();
		//pick up point pcl to cgal vector object
		for (; start_id != dest->end(); ++start_id) {
			Point p(start_id->x, start_id->y, start_id->z);
			points.push_back(p);
		}
		std::cout << points.size() << "is input points size of the given point cloud file" << std::endl;
		std::vector<std::size_t> indices(points.size());
		// vector to array

		for (std::size_t i = 0; i < points.size(); ++i)
		{
			indices[i] = i;
		}
		double cell_size = 0.025;
		std::vector<std::size_t>::iterator end;
		end = CGAL::grid_simplify_point_set(indices.begin(),
			indices.end(), CGAL::make_property_map(points),
			cell_size);
		std::size_t k = end - indices.begin();
		std::cerr << "Keep " << k << " of " << indices.size() << "indices" << std::endl;
		std::vector<Point> tmp_points(k);
		for (std::size_t i = 0; i < k; ++i)
		{
			tmp_points[i] = points[indices[i]];
		}
		points.swap(tmp_points);

		std::cout << points.size() << " points after the simplification" << std::endl;

		// CGAL geometry
		Polyhedron Pol;

		// polyhedron construction from 3d cloud points
		Polyhedron P;
		P.make_tetrahedron();
		CGAL::set_ascii_mode(std::cout);
		std::copy(P.points_begin(), P.points_end(), std::ostream_iterator<Point>(std::cout, "\n"));

		// no of partitioning
		std::cout << "Enter no of partition levels:\n";
		unsigned int d;
		std::cin >> d;
		std::cout << "You enter  partition number is:" << d << std::endl;

		// convex hull computation
		CGAL::convex_hull_3(points.begin(), points.end(), Pol);
		P = Pol;
		// surface sub-division
		Subdivision_method_3::CatmullClark_subdivision(P, d);

		// save outputs
		std::cout << "saving output to .off file " << std::endl;
		std::ofstream f("out_point1.off");
		CGAL::write_off(f, Pol);

		std::ofstream f1("out_point2.off");
		CGAL::write_off(f1, P);

		// display point dat objects i.e. visualization cloud data...
		boost::shared_ptr<visualization::PCLVisualizer> cloudView(new visualization::PCLVisualizer("3D Viewer"));
		cloudView->initCameraParameters();

		int v1(0);
		cloudView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		cloudView->setBackgroundColor(0.0, 0.0, 0.0, v1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1(100, 200, 210);
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LUT, 1.0f, "Cloud", v1);
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0f, "Cloud", v1);
		cloudView->addPointCloud<pcl::PointXYZRGB>(src, "SrcCloud", v1);

		int v2(0);
		cloudView->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		cloudView->setBackgroundColor(0.0, 0.0, 0.0, v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(150, 255, 250);
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "OutCloud");
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LUT, 1.0f, "OutCloud", v2);
		cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0f, "OutCloud", v2);
		cloudView->addPointCloud<pcl::PointXYZRGB>(outcloud, "OutCloud", v2);
		cloudView->setPosition(120, 10);
		cloudView->setSize(900, 850);
		cloudView->setShowFPS(true);
		cloudView->setWindowName("Point reduction Filter");
		cloudView->resetCameraViewpoint();
		cloudView->resetCamera();

		while (!cloudView->wasStopped())
		{
			cloudView->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}

		cloudView->updateCamera();
		cloudView->removePolygonMesh("src");
		cloudView->close();
		f.close();
	}
	catch (const std::exception& errob)
	{
		std::cerr << "Error found" << errob.what() << std::endl;
	}

	return EXIT_SUCCESS;
}