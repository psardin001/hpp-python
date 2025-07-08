//
// Copyright (c) 2025, CNRS
// Authors: Florent Lamiraux
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

#include <boost/python.hpp>
#include <../src/pyhpp/manipulation/problem.hh>
#include <../src/pyhpp/manipulation/device.hh>
#include <../src/pyhpp/manipulation/graph.hh>
#include <hpp/core/problem.hh>

using namespace boost::python;

namespace pyhpp {
namespace manipulation {

Problem::Problem(const PyWDevicePtr_t& robot)
    : obj(hpp::manipulation::Problem::create(robot->obj)) {
    }

Problem::Problem(const hpp::manipulation::ProblemPtr_t& object) { obj = object; }

void Problem::constraintGraph(const PyWGraphPtr_t &graph) {
    obj->constraintGraph(graph->obj);
}

PyWGraphPtr_t Problem::constraintGraph() const {
    pyhpp::manipulation::PyWGraph* graph = new PyWGraph(obj->constraintGraph());
    return std::shared_ptr<PyWGraph>(graph);
}

void Problem::checkProblem() const {
    obj->checkProblem();
}

ConfigurationShooterPtr_t Problem::configurationShooter() const {
  return obj->configurationShooter();
}


// PathValidationPtr_t Problem::pathValidation() const {
//     return obj->pathValidation();
// }

// void Problem::pathValidation(const PathValidationPtr_t &pathValidation) {
//     obj->pathValidation(pathValidation);
// }

// SteeringMethodPtr_t Problem::manipulationSteeringMethod() const {
//     return obj->manipulationSteeringMethod();
// }

// PathValidationPtr_t Problem::pathValidationFactory() const {
//     return obj->pathValidationFactory();
// }

// void Problem::setPathValidationFactory(const core::PathValidationBuilder_t &factory, const value_type &tol) {
//     obj->setPathValidationFactory(factory, tol);
// }

// static void declareParameter(const ParameterDescription &desc) {
//     Problem::declareParameter(desc);
// }

// static const Container<ParameterDescription> & parameterDescriptions() {
//     return Problem::parameterDescriptions();
// }

// static const ParameterDescription & parameterDescription(const std::string &name) {
//     return Problem::parameterDescription(name);
// }

void exposeProblem() {
  class_<Problem>("Problem", init<const PyWDevicePtr_t&>())
  .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(Problem, constraintGraph, PyWGraphPtr_t)
  .PYHPP_DEFINE_METHOD(Problem, checkProblem)
  .PYHPP_DEFINE_METHOD(Problem, configurationShooter)
  // .PYHPP_DEFINE_GETTER_SETTER_CONST_REF(Problem, pathValidation, PathValidationPtr_t)
  // .PYHPP_DEFINE_METHOD(Problem, manipulationSteeringMethod)
  // .PYHPP_DEFINE_METHOD(Problem, pathValidationFactory)
  // .PYHPP_DEFINE_METHOD(Problem, setPathValidationFactory)
  // .PYHPP_DEFINE_METHOD_STATIC(Problem, declareParameter)
  // .PYHPP_DEFINE_METHOD_STATIC(Problem, parameterDescriptions)
  // .PYHPP_DEFINE_METHOD_STATIC(Problem, parameterDescription)
  ;
}
}  // namespace manipulation
}  // namespace pyhpp
