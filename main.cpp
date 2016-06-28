#include <iostream>
#include "main.h"



using namespace std;


namespace operations_research {


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRP(Matrix &matrix, int64 num_v, int64 lns, int64 tm) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");


        RoutingSearchParameters parameters = routing.DefaultSearchParameters();


        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(lns);
        parameters.set_time_limit_ms(tm);

        // parameters.set_time_limit_ms(240 * 1000);
        parameters.set_guided_local_search_lambda_coefficient(0.1);


        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;

        // parameters.local_search_metaheuristic();


        // Setting depot
        //CHECK_GT(FLAGS_depot, 0) << " Because we use the" << " TSPLIB convention, the depot id must be > 0";
        //RoutingModel::NodeIndex depot(FLAGS_depot -1);
        //routing.SetDepot(depot);

        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }

    }  //  void  VRPSolver (CVRPData & data)


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRPTW(Matrix &matrix, int64 num_v, int64 lns, int64 tm) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), true, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();


        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar *const cumul_var = routing.CumulVar(index, "time");
            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }


        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(lns);
        parameters.set_time_limit_ms(tm);
        parameters.set_guided_local_search_lambda_coefficient(0.1);

        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;


        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }



        //  void  VRPSolver (CVRPData & data)
    }

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTWMD(Matrix &matrix, int64 num_v,
                                                                std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex >> &depots, int64 lns, int64 tm) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        int size = matrix.getSize();

        RoutingModel routing(size, num_v, depots);


        // routing.SetStartEnd()

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), true, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();


        for (int64 i = 0; i < matrix.getSize(); ++i) {
            int64 index = i;
            IntVar *const cumul_var = routing.CumulVar(index, "time");
            cumul_var->SetMin(matrix.getTimeWindow(i).first);
            cumul_var->SetMax(matrix.getTimeWindow(i).second);
        }


        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(lns);
        parameters.set_time_limit_ms(tm);
        parameters.set_guided_local_search_lambda_coefficient(0.1);

        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;


        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                result.push_back(routing.IndexToNode(routing.End(vehicle_nbr)).value());
                cvrp_result.push_back(result);
            }

            std::cout << "Debug info: " << routing.solver()->DebugString() << std::endl;

            std::cout << "Wall tim:e " << routing.solver()->wall_time() << std::endl;
            std::cout << "Failures: " << routing.solver()->failures() << std::endl;
            std::cout << "Branches: " << routing.solver()->branches() << std::endl;

            std::cout << "Parameters Debug String: " << parameters.DebugString() << std::endl;


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }


    }

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTWPD(Matrix &matrix, int64 num_v,
                                                                std::vector<RoutingModel::NodeIndex> pickups,
                                                                std::vector<RoutingModel::NodeIndex> deliveries, int64 lns, int64 tm) {


        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), true, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();





//        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
//            int64 index = routing.NodeToIndex(i);
//            IntVar *const cumul_var = routing.CumulVar(index, "time");
//            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
//            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
//        }

        const RoutingDimension &time_dimension = routing.GetDimensionOrDie("time");
        Solver *const solver = routing.solver();

        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); i++) {

            const int64 index = routing.NodeToIndex(i);
            if (pickups[i.value()] == 0) {
                if (deliveries[i.value()] == 0) {
                    routing.SetDepot(i);
                } else {

                    const int64 delivery_index = routing.NodeToIndex(deliveries[i.value()]);
                    solver->AddConstraint(solver->MakeEquality(
                            routing.VehicleVar(index), routing.VehicleVar(delivery_index)));
                    solver->AddConstraint(
                            solver->MakeLessOrEqual(time_dimension.CumulVar(index),
                                                    time_dimension.CumulVar(delivery_index)));
                    routing.AddPickupAndDelivery(i, deliveries[i.value()]);

                }


            }

            IntVar* const cumul = time_dimension.CumulVar(index);
            cumul->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul->SetMax(matrix.getTimeWindow(i.value()).second);

        }




            parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
            parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
            parameters.set_lns_time_limit_ms(lns);
            parameters.set_time_limit_ms(tm);
            parameters.set_guided_local_search_lambda_coefficient(0.1);


            const Assignment *solution = routing.SolveWithParameters(parameters);

            //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

            // INSPECT SOLUTION
            if (solution != NULL) {

                // Solution cost.
                std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
                // Inspect solution.
                std::string route;
                for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                    route = "";
                    result.clear();
                    for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                         node = solution->Value(routing.NextVar(node))) {
                        route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                        result.push_back(routing.IndexToNode(node).value());
                    }
                    route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                    std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                    cvrp_result.push_back(result);
                }


                return cvrp_result;

            } else {
                    LG << "No solution found.";
            }


        }




    }


    bool diagonalTest(std::vector<std::vector<int64>> vec) {

        for (int i = 0; i < vec.size(); i++) {
            for (int j = 0; j < vec.size(); j++) {
                if (i == j && vec[i][j] != 0) {
                    return false;
                }
                else continue;
            }
        }
        return true;
    }


