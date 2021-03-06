//
// Created by veter on 07.06.16.
//

#ifndef CVRP_MAIN_H
#define CVRP_MAIN_H

#include "constraint_solver/routing.h"
#include "matr.h"

namespace operations_research {

    class CVRPTWSolver{

    public:

      static  std::vector<std::vector<int64>> SolveCVRP (Matrix& matrix, int64 num_v, int64 lns, int64 tm);

      static  std::vector<std::vector<int64>>  SolveCVRPTW (Matrix& matrix, int64 num_v, int64 lns, int64 tm);

        static   std::vector<std::vector<int64>> SolveCVRPTWMD (Matrix& matrix, int64 num_v,
                                                                std::vector<pair<RoutingModel::NodeIndex,
                                                                        RoutingModel::NodeIndex>> &depots, int64 lns, int64 tm);



        static  std::vector<std::vector<int64>> SolveCVRPTWPD(Matrix& matrix, int64 num_v,  std::vector<RoutingModel::NodeIndex> pickups,
                                                              std::vector<RoutingModel::NodeIndex> deliveries, int64 lns, int64 tm);
    };


}



#endif //CVRP_MAIN_H
