#include <graph_mapping_tools/graph_slam.hpp>

#include <boost/format.hpp>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace g2o {
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
}

namespace graph_mapping_tools {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM(const std::string& solver_type) {
  graph.reset(new g2o::SparseOptimizer());

  std::cout << "construct solver... " << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

  robust_kernel_factory = g2o::RobustKernelFactory::instance();
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() {
  graph.reset();
}


g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  vertex->setEstimate(pose);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
  Eigen::Matrix<double, 6, 1> m;
  m.head<3>() = direction;
  m.tail<3>() = measurement;

  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  edge->setMeasurement(m);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

void GraphSLAM::add_robust_kernel(g2o::OptimizableGraph::Edge* edge, const std::string& kernel_type, double kernel_size) {
  if(kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if(kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
}

void GraphSLAM::optimize(int num_iterations) {
  if(graph->edges().size() < 10) {
    return;
  }

  std::cout << std::endl;
  std::cout << "<<<---factor graph optimization --->>>" << std::endl;
  std::cout << "states: " << graph->vertices().size() << "   constraints: " << graph->edges().size() << std::endl;
  std::cout << "start optimizing... " << std::flush;

  graph->initializeOptimization();
  graph->computeInitialGuess();
  graph->computeActiveErrors();
  graph->setVerbose(false);

  double chi2 = graph->chi2();

  auto t1 = ros::Time::now();
  int iterations = graph->optimize(num_iterations);

  auto t2 = ros::Time::now();
  std::cout << "finish optimization..." << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  // std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "used time for optimization: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
}

void GraphSLAM::save(const std::string& filename) {
  std::ofstream ofs(filename);
  graph->save(ofs);
}

}
