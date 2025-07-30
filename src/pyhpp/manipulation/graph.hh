//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux, Paul Sardin
//
// This file is part of hpp-python
// hpp-python is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-python is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-python  If not, see
// <http://www.gnu.org/licenses/>.

#include <pyhpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/graph.hh>

namespace pyhpp {
namespace manipulation {

typedef hpp::manipulation::size_type size_type;
typedef hpp::manipulation::value_type value_type;
typedef hpp::manipulation::graph::GraphComponent GraphComponent;
typedef hpp::manipulation::graph::GraphComponentPtr_t GraphComponentPtr_t;
typedef hpp::manipulation::ProblemPtr_t ProblemPtr_t;
typedef hpp::manipulation::graph::Edge Edge;
typedef hpp::manipulation::graph::EdgePtr_t EdgePtr_t;
typedef hpp::manipulation::graph::State State;
typedef hpp::manipulation::graph::StatePtr_t StatePtr_t;
typedef hpp::manipulation::graph::GraphPtr_t GraphPtr_t;

/// Result structure for constraint operations
struct ConstraintResult {
  bool success;
  Configuration_t configuration;
  value_type error;

  ConstraintResult() : success(false), configuration(), error(0.0) {}
  ConstraintResult(bool s, const Configuration_t& config, value_type err)
      : success(s), configuration(config), error(err) {}
};

/// Python wrapper for State
struct PyWState {
  StatePtr_t obj;
  PyWState(const StatePtr_t& object);
};
typedef std::shared_ptr<PyWState> PyWStatePtr_t;

/// Python wrapper for Edge  
struct PyWEdge {
  EdgePtr_t obj;
  PyWEdge(const EdgePtr_t& object);
};
typedef std::shared_ptr<PyWEdge> PyWEdgePtr_t;

/// Python wrapper for Graph
struct PyWGraph {
  typedef hpp::constraints::ImplicitPtr_t ImplicitPtr_t;
  typedef hpp::constraints::NumericalConstraints_t NumericalConstraints_t;

  // Member variables
  GraphPtr_t obj;
  PyWDevicePtr_t robot;
  PyWProblemPtr_t problem;

  // Constructors
  PyWGraph(const GraphPtr_t& object);
  PyWGraph(const std::string& name, const PyWDevicePtr_t& d, 
           const PyWProblemPtr_t& problem);

  // Configuration methods
  void maxIterations(size_type iterations);
  size_type maxIterations() const;
  void errorThreshold(const value_type& threshold);
  value_type errorThreshold() const;

  // Graph construction
  PyWStatePtr_t createState(const std::string& nodeName, bool waypoint, 
                           int priority);
  PyWEdgePtr_t createTransition(PyWStatePtr_t nodeFrom, PyWStatePtr_t nodeTo,
                         const std::string& transitionName, int w,
                         PyWStatePtr_t isInState);
  PyWEdgePtr_t createWaypointTransition(PyWStatePtr_t nodeFrom, PyWStatePtr_t nodeTo,
                                 const std::string& edgeName, int nb, int w, 
                                 PyWStatePtr_t isInState, bool automaticBuilder);
  PyWEdgePtr_t createLevelSetTransition(PyWStatePtr_t nodeFrom, PyWStatePtr_t nodeTo,
                                 const std::string& edgeName, int w,
                                 PyWStatePtr_t isInState);

  // Edge/State management
  void setContainingNode(PyWEdgePtr_t edge, PyWStatePtr_t node);
  std::string getContainingNode(PyWEdgePtr_t edge);
  void setShort(PyWEdgePtr_t edge, bool isShort);
  bool isShort(PyWEdgePtr_t edge);
  void getNodesConnectedByTransition(PyWEdgePtr_t edge, std::string& nodeFrom, 
                              std::string& nodeTo);
  void setWeight(PyWEdgePtr_t edge, int weight);
  size_t getWeight(PyWEdgePtr_t edge);
  void setWaypoint(PyWEdgePtr_t waypointEdge, int index, 
                           PyWEdgePtr_t edge, PyWStatePtr_t state);
  void addNumericalConstraintsToGraph(const boost::python::list& py_constraints);
  // State queries
  std::string getState(ConfigurationIn_t input);

  // Constraint management
  void addNumericalConstraint(PyWStatePtr_t node, const ImplicitPtr_t& constraint);
  void addNumericalConstraintsToState(PyWStatePtr_t component, 
                              const boost::python::list& py_constraints);
  void addNumericalConstraintsToTransition(PyWEdgePtr_t component, 
                              const boost::python::list& py_constraints);
  void addNumericalConstraintsForPath(PyWStatePtr_t component, 
                                     const boost::python::list& py_constraints);
  void resetConstraints(PyWStatePtr_t component);
  void registerConstraints(const ImplicitPtr_t& constraint,
                          const ImplicitPtr_t& complement,
                          const ImplicitPtr_t& both);

  boost::python::tuple createPlacementConstraint(const std::string& name,
                                                 const boost::python::list& py_surface1,
                                                 const boost::python::list& py_surface2,
                                                 const value_type& margin);
  ImplicitPtr_t createPrePlacementConstraint(const std::string& name,
                                                 const boost::python::list& py_surface1,
                                                 const boost::python::list& py_surface2,
                                                 const value_type& width,
                                                 const value_type& margin);
  boost::python::tuple createPlacementConstraint1(
    const std::string& name, const boost::python::list& py_surface1,
    const boost::python::list& py_surface2, const value_type& margin);
boost::python::tuple createPlacementConstraint2(
    const std::string& name, const boost::python::list& py_surface1,
    const boost::python::list& py_surface2);
ImplicitPtr_t createPrePlacementConstraint1(
    const std::string& name, const boost::python::list& py_surface1,
    const boost::python::list& py_surface2, const value_type& width, const value_type& margin);
ImplicitPtr_t createPrePlacementConstraint2(
    const std::string& name, const boost::python::list& py_surface1,
    const boost::python::list& py_surface2, const value_type& width);

boost::python::list getNumericalConstraintsForState(PyWStatePtr_t component);
boost::python::list getNumericalConstraintsForEdge(PyWEdgePtr_t component);
boost::python::list getNumericalConstraintsForGraph();

  // Configuration error checking
  bool getConfigErrorForState(PyWStatePtr_t component, ConfigurationIn_t input, 
                             hpp::core::vector_t& error);
  bool getConfigErrorForTransition(PyWEdgePtr_t edge, ConfigurationIn_t input, 
                            hpp::core::vector_t& error);
  bool getConfigErrorForTransitionLeaf(ConfigurationIn_t leafConfig, 
                                ConfigurationIn_t config, 
                                const PyWEdgePtr_t& edge, 
                                hpp::core::vector_t& error) const;
  bool getConfigErrorForTransitionTarget(ConfigurationIn_t leafConfig, 
                                  ConfigurationIn_t config, 
                                  const PyWEdgePtr_t& edge, 
                                  hpp::core::vector_t& error) const;

  // Constraint application
  ConstraintResult applyStateConstraints(PyWStatePtr_t state, 
                                        ConfigurationIn_t input);
  ConstraintResult applyLeafConstraints(PyWEdgePtr_t transition, 
                                       ConfigurationIn_t q_rhs,
                                       ConfigurationIn_t input);
  ConstraintResult generateTargetConfig(PyWEdgePtr_t transition, 
                                       ConfigurationIn_t q_rhs,
                                       ConfigurationIn_t input);

  // Level set edges
  void addLevelSetFoliation(PyWEdgePtr_t edge, 
                           const boost::python::list& condNC,
                           const boost::python::list& paramNC);

  // Security margins and collision
  boost::python::list getSecurityMarginMatrixForTransition(PyWEdgePtr_t edge);
  void setSecurityMarginForTransition(PyWEdgePtr_t edge, const char* joint1,
                               const char* joint2, double margin);
  boost::python::list getRelativeMotionMatrix(PyWEdgePtr_t edge);
  void removeCollisionPairFromTransition(PyWEdgePtr_t edge, const char* joint1, 
                                  const char* joint2);

  // Subgraph management
  void createSubGraph(const char* subgraphName, hpp::core::RoadmapPtr_t roadmap);
  void setTargetNodeList(const boost::python::list& nodes);

  // Display and debugging
  std::string displayStateConstraints(PyWStatePtr_t state);
  std::string displayTransitionConstraints(PyWEdgePtr_t edge);
  std::string displayTransitionTargetConstraints(PyWEdgePtr_t edge);
  void display(const char* filename);

  // Initialization
  void initialize();
};

}  // namespace manipulation
}  // namespace pyhpp