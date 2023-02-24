#pragma once

#include <type_traits>
#include <limits>

#include "AlgorithmImpl.hpp"

#include "sources/TaskModel/Tasks.h"

template <typename GenotypeType>
class NumericAlgorithm : public Algorithm<GenotypeType>
{
public:
    NumericAlgorithm(unsigned int _moesPerGen, unsigned int _dimensions = 1, std::vector<GenotypeType> _range = {std::numeric_limits<GenotypeType>::lowest(), std::numeric_limits<GenotypeType>::max()});

    std::vector<GenotypeType> getRandomGenotype() override;
    std::vector<GenotypeType> getRandomGenotype(rt_num_opt::TaskSet &tasks);

protected:
    unsigned int m_moesPerGen,
        m_dimensions;

    std::vector<GenotypeType> m_range;
    std::uniform_real_distribution<> dist_genotype;
};

template <typename GenotypeType>
NumericAlgorithm<GenotypeType>::NumericAlgorithm(unsigned int _moesPerGen, unsigned int _dimensions, std::vector<GenotypeType> _range)
    : Algorithm<GenotypeType>(),
      m_moesPerGen(_moesPerGen),
      m_dimensions(_dimensions),
      m_range(_range),
      dist_genotype(_range[0], _range[1])
{
    static_assert(std::is_arithmetic<GenotypeType>::value, "NumericAlgorithm only works with arithmetic types");
}

template <typename GenotypeType>
std::vector<GenotypeType> NumericAlgorithm<GenotypeType>::getRandomGenotype()
{
    std::vector<GenotypeType> genotype;
    genotype.reserve(m_dimensions);

    while (genotype.size() < m_dimensions)
        genotype.push_back(dist_genotype(Algorithm<GenotypeType>::m_generator));

    return genotype;
}

template <typename GenotypeType>
std::vector<GenotypeType> NumericAlgorithm<GenotypeType>::getRandomGenotype(rt_num_opt::TaskSet &tasks)
{
    std::vector<GenotypeType> genotype;
    genotype.reserve(m_dimensions);

    // while (genotype.size() < m_dimensions)
    //     genotype.push_back(dist_genotype(Algorithm<GenotypeType>::m_generator));
    srand(time(0));
    for (size_t i = 0; i < tasks.size(); i++)
    {
        genotype.push_back((GenotypeType)(rt_num_opt::RandRange(tasks[i].executionTimeOrg, tasks[i].executionTimeOrg * 2)));
    }
    return genotype;
}
