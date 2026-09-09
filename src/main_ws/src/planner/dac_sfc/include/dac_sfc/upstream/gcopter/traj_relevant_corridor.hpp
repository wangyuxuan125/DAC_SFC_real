#ifndef TRAJ_RELEVANT_CORRIDOR_HPP
#define TRAJ_RELEVANT_CORRIDOR_HPP

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace traj_relevant
{

enum class CandidateSelectionMode
{
    BATCH_SET_COVER = 0,
    ACTIVE_WITNESS = 1
};

struct CompactCorridorOptions
{
    CandidateSelectionMode candidate_selection_mode =
        CandidateSelectionMode::BATCH_SET_COVER;

    // Maximum additional expansion beyond the protected seed segment.
    // Easy MINCO deformation directions approach this value.
    double max_extra_radius =
        3.0;

    // Low-utility directions only receive this fraction of
    // max_extra_radius.
    //
    // beta = 0.25:
    //     insensitive direction -> 0.25 * max radius
    //     easy direction        -> 1.00 * max radius
    double min_extra_ratio =
        0.25;

    // Guaranteed radius around [a,b].
    //
    // If adjacent segments share a junction q, both corridors
    // contain B(q, overlap_radius).
    double overlap_radius =
        0.01;

    bool metric_enabled =
        false;

    // CSGN deformation utility S.
    //
    // Large eigenvalue:
    //     MINCO can usefully deform along that eigenvector.
    Eigen::Matrix3d deformation_utility =
        Eigen::Matrix3d::Identity();

    double epsilon =
        1.0e-6;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CompactCorridorDiagnostics
{
    int input_obstacle_count =
        0;

    int local_obstacle_count =
        0;

    int domain_face_count =
        0;

    int candidate_count =
        0;

    int active_witness_rounds =
        0;

    int generated_candidate_count =
        0;

    std::int64_t obstacle_face_tests =
        0;
    
    std::int64_t witness_distance_tests =
        0;

    int greedy_obstacle_face_count =
        0;

    int selected_obstacle_face_count =
        0;

    int redundancy_removed =
        0;

    int total_face_count =
        0;

    bool metric_valid =
        false;

    bool anisotropic_domain =
        false;

    bool overlap_guaranteed =
        false;

    bool safety_verified =
        false;

    Eigen::Vector3d utility_eigenvalues =
        Eigen::Vector3d::Ones();

    Eigen::Vector3d extra_radii =
        Eigen::Vector3d::Zero();

    double mean_metric_damage =
        0.0;

    double min_metric_damage =
        0.0;

    double max_metric_damage =
        0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CandidateFace
{
    Eigen::Vector4d plane =
        Eigen::Vector4d::Zero();

    std::vector<int>
        covered_obstacles;

    int source_obstacle =
        -1;

    double metric_damage =
        0.0;

    double seed_obstacle_clearance =
        0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using CandidateFaces =
    std::vector<
        CandidateFace,
        Eigen::aligned_allocator<
            CandidateFace>>;

// ================================================================
// Trajectory-relevant compact polytope for one seed segment [a,b].
//
// This is deliberately NOT FIRI:
//
//   - no MVIE,
//   - no maximum-volume iteration,
//   - no nearest-obstacle inflation loop,
//   - no candidate_pool_size,
//   - no face_count_weight.
//
// Construction:
//
//   1. Build a CSGN-aligned bounded domain.
//   2. Do not fully inflate MINCO-insensitive directions.
//   3. Generate metric-aware separating planes in batch.
//   4. Solve an approximate minimum-face set-cover.
//   5. Reverse-delete redundant selected faces.
//   6. Verify the complete local obstacle cloud.
// ================================================================
inline bool buildCompactSegmentPolytope(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &lowCorner,
    const Eigen::Vector3d &highCorner,
    const Eigen::Vector3d &a,
    const Eigen::Vector3d &b,
    Eigen::MatrixX4d &hPoly,
    const CompactCorridorOptions &options =
        CompactCorridorOptions(),
    CompactCorridorDiagnostics *diagnostics =
        nullptr)
{
    CompactCorridorDiagnostics localDiagnostics;

    localDiagnostics.input_obstacle_count =
        static_cast<int>(
            points.size());

    const double epsilon =
        std::max(
            options.epsilon,
            1.0e-12);

    const double overlapRadius =
        std::max(
            0.0,
            options.overlap_radius);

    const double maxExtraRadius =
        std::max(
            0.0,
            options.max_extra_radius);

    const double minExtraRatio =
        std::min(
            1.0,
            std::max(
                0.0,
                options.min_extra_ratio));

    // ------------------------------------------------------------
    // Protected seed capsule must itself lie inside the global map.
    // ------------------------------------------------------------
    for (int axis = 0;
         axis < 3;
         ++axis)
    {
        if (std::max(a(axis), b(axis)) +
                overlapRadius >
            highCorner(axis) +
                epsilon ||
            std::min(a(axis), b(axis)) -
                overlapRadius <
            lowCorner(axis) -
                epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    // ------------------------------------------------------------
    // Validate S and compute its eigensystem.
    //
    // S = U diag(mu) U^T
    //
    // Large mu:
    //     easy / useful MINCO deformation direction.
    // ------------------------------------------------------------
    Eigen::Matrix3d utility =
        0.5 *
        (options.deformation_utility +
         options.deformation_utility.transpose());

    Eigen::Matrix3d eigenvectors =
        Eigen::Matrix3d::Identity();

    Eigen::Vector3d eigenvalues =
        Eigen::Vector3d::Ones();

    if (options.metric_enabled &&
        utility.allFinite())
    {
        Eigen::SelfAdjointEigenSolver<
            Eigen::Matrix3d>
            solver(
                utility);

        if (solver.info() ==
                Eigen::Success &&
            solver.eigenvalues()
                    .minCoeff() >
                epsilon)
        {
            localDiagnostics.metric_valid =
                true;

            eigenvectors =
                solver.eigenvectors();

            eigenvalues =
                solver.eigenvalues();
        }
    }

    if (!localDiagnostics.metric_valid)
    {
        utility.setIdentity();

        eigenvectors.setIdentity();

        eigenvalues.setOnes();
    }

    localDiagnostics.utility_eigenvalues =
        eigenvalues;

    // ------------------------------------------------------------
    // Convert the CSGN spectrum into directional expansion.
    //
    // g_j in [0,1]:
    //
    //     g_j = 0 -> MINCO-insensitive direction
    //     g_j = 1 -> easiest deformation direction
    //
    // Extra radius:
    //
    //   e_j = R [ beta + (1-beta) g_j ].
    //
    // If the metric is unavailable or isotropic, all directions
    // receive the original full range R.
    // ------------------------------------------------------------
    Eigen::Vector3d normalizedUtility =
        Eigen::Vector3d::Ones();

    if (localDiagnostics.metric_valid)
    {
        const Eigen::Vector3d logEigenvalues =
            eigenvalues.array().log();

        const double logMin =
            logEigenvalues.minCoeff();

        const double logMax =
            logEigenvalues.maxCoeff();

        const double logSpan =
            logMax -
            logMin;

        if (logSpan >
            epsilon)
        {
            normalizedUtility =
                (logEigenvalues.array() -
                 logMin) /
                logSpan;

            localDiagnostics.anisotropic_domain =
                true;
        }
    }

    const Eigen::Vector3d extraRadii =
        maxExtraRadius *
        (minExtraRatio *
             Eigen::Vector3d::Ones() +
         (1.0 -
          minExtraRatio) *
             normalizedUtility);

    localDiagnostics.extra_radii =
        extraRadii;

    // ------------------------------------------------------------
    // Build the CSGN-aligned finite domain.
    //
    // In eigenvector direction u_j:
    //
    // half width =
    //
    //   0.5 |u_j^T (b-a)|
    //   + overlap_radius
    //   + e_j.
    //
    // Therefore insensitive directions are deliberately NOT
    // maximally inflated.
    // ------------------------------------------------------------
    const Eigen::Vector3d center =
        0.5 *
        (a + b);

    const Eigen::Vector3d segment =
        b - a;

    Eigen::Vector3d halfWidths =
        Eigen::Vector3d::Zero();

    std::vector<
        Eigen::Vector4d,
        Eigen::aligned_allocator<
            Eigen::Vector4d>>
        fixedPlanes;

    fixedPlanes.reserve(
        12);

    for (int directionId = 0;
         directionId < 3;
         ++directionId)
    {
        const Eigen::Vector3d direction =
            eigenvectors.col(
                directionId);

        halfWidths(directionId) =
            0.5 *
                std::abs(
                    direction.dot(
                        segment)) +
            overlapRadius +
            extraRadii(directionId);

        const double centerProjection =
            direction.dot(
                center);

        const double upper =
            centerProjection +
            halfWidths(directionId);

        const double lower =
            centerProjection -
            halfWidths(directionId);

        Eigen::Vector4d upperPlane;
        upperPlane.head<3>() =
            direction;
        upperPlane(3) =
            -upper;

        Eigen::Vector4d lowerPlane;
        lowerPlane.head<3>() =
            -direction;
        lowerPlane(3) =
            lower;

        fixedPlanes.push_back(
            upperPlane);

        fixedPlanes.push_back(
            lowerPlane);
    }

    // ------------------------------------------------------------
    // Clip against global map bounds only when the oriented domain
    // reaches beyond the map.
    // ------------------------------------------------------------
    for (int axis = 0;
         axis < 3;
         ++axis)
    {
        double worldHalfWidth =
            0.0;

        for (int directionId = 0;
             directionId < 3;
             ++directionId)
        {
            worldHalfWidth +=
                std::abs(
                    eigenvectors(
                        axis,
                        directionId)) *
                halfWidths(
                    directionId);
        }

        const double domainHigh =
            center(axis) +
            worldHalfWidth;

        const double domainLow =
            center(axis) -
            worldHalfWidth;

        if (domainHigh >
            highCorner(axis) +
                epsilon)
        {
            Eigen::Vector4d plane =
                Eigen::Vector4d::Zero();

            plane(axis) =
                1.0;

            plane(3) =
                -highCorner(axis);

            fixedPlanes.push_back(
                plane);
        }

        if (domainLow <
            lowCorner(axis) -
                epsilon)
        {
            Eigen::Vector4d plane =
                Eigen::Vector4d::Zero();

            plane(axis) =
                -1.0;

            plane(3) =
                lowCorner(axis);

            fixedPlanes.push_back(
                plane);
        }
    }

    localDiagnostics.domain_face_count =
        static_cast<int>(
            fixedPlanes.size());

    Eigen::MatrixX4d fixedH(
        fixedPlanes.size(),
        4);

    for (int faceId = 0;
         faceId <
             static_cast<int>(
                 fixedPlanes.size());
         ++faceId)
    {
        fixedH.row(faceId) =
            fixedPlanes[faceId]
                .transpose();
    }

    // ------------------------------------------------------------
    // Crop obstacle samples to the trajectory-relevant domain.
    //
    // Points outside this domain are already excluded by a fixed
    // domain halfspace and therefore need no obstacle-generated face.
    // ------------------------------------------------------------
    std::vector<
        Eigen::Vector3d,
        Eigen::aligned_allocator<
            Eigen::Vector3d>>
        localObstacles;

    localObstacles.reserve(
        points.size());

    for (const Eigen::Vector3d &point :
         points)
    {
        const Eigen::Vector4d pointH(
            point(0),
            point(1),
            point(2),
            1.0);

        if ((fixedH *
             pointH)
                .maxCoeff() <=
            epsilon)
        {
            localObstacles.push_back(
                point);
        }
    }

    const int obstacleCount =
        static_cast<int>(
            localObstacles.size());

    localDiagnostics.local_obstacle_count =
        obstacleCount;

    if (obstacleCount == 0)
    {
        hPoly =
            fixedH;

        localDiagnostics.total_face_count =
            static_cast<int>(
                hPoly.rows());

        localDiagnostics.overlap_guaranteed =
            true;

        localDiagnostics.safety_verified =
            true;

        if (diagnostics != nullptr)
        {
            *diagnostics =
                localDiagnostics;
        }

        return true;
    }

    // ------------------------------------------------------------
    // W = S^{-1}.
    //
    // Large utility in S becomes low normal weight in W.
    // Candidate normals therefore avoid pointing along MINCO's
    // easy deformation directions whenever geometry permits.
    // ------------------------------------------------------------
    Eigen::Matrix3d inverseUtility =
        Eigen::Matrix3d::Identity();

    if (localDiagnostics.metric_valid)
    {
        inverseUtility =
            eigenvectors *
            eigenvalues
                .cwiseInverse()
                .asDiagonal() *
            eigenvectors.transpose();
    }

    const double segmentMetricNormSquared =
        segment.dot(
            inverseUtility *
            segment);

    CandidateFaces candidates;

    std::vector<unsigned char>
        selected;

    std::vector<int>
        selectedCandidateIds;

    if (options.candidate_selection_mode ==
        CandidateSelectionMode::BATCH_SET_COVER)
    {
        candidates.reserve(
            obstacleCount);

        // ------------------------------------------------------------
        // Candidate generation.
        //
        // For obstacle o:
        //
        //   q = argmin_{x in [a,b]}
        //           (o-x)^T W (o-x)
        //
        //   n ~ W(o-q).
        //
        // Candidate plane is placed midway between the protected seed
        // capsule support and the obstacle support.
        //
        // This is deliberately independent of any MVIE state.
        // ------------------------------------------------------------
        for (int obstacleId = 0;
             obstacleId <
                 obstacleCount;
             ++obstacleId)
        {
            const Eigen::Vector3d obstacle =
                localObstacles[
                    obstacleId];

            double interpolation =
                0.0;

            if (segmentMetricNormSquared >
                epsilon *
                    epsilon)
            {
                interpolation =
                    segment.dot(
                        inverseUtility *
                        (obstacle - a)) /
                    segmentMetricNormSquared;

                interpolation =
                    std::max(
                        0.0,
                        std::min(
                            1.0,
                            interpolation));
            }

            const Eigen::Vector3d projection =
                a +
                interpolation *
                    segment;

            Eigen::Vector3d normal =
                inverseUtility *
                (obstacle -
                 projection);

            const double normalNorm =
                normal.norm();

            if (!std::isfinite(
                    normalNorm) ||
                normalNorm <=
                    epsilon)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            normal /=
                normalNorm;

            const double protectedSupport =
                std::max(
                    normal.dot(a),
                    normal.dot(b)) +
                overlapRadius;

            const double obstacleSupport =
                normal.dot(
                    obstacle);

            const double supportGap =
                obstacleSupport -
                protectedSupport;

            if (!std::isfinite(
                    supportGap) ||
                supportGap <=
                    2.0 *
                        epsilon)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            const double threshold =
                0.5 *
                (protectedSupport +
                 obstacleSupport);

            CandidateFace candidate;

            candidate.source_obstacle =
                obstacleId;

            candidate.plane.head<3>() =
                normal;

            candidate.plane(3) =
                -threshold;

            candidate.seed_obstacle_clearance =
                0.5 *
                supportGap;

            const double rayleigh =
                normal.dot(
                    utility *
                    normal);

            if (std::isfinite(rayleigh) &&
                rayleigh >
                    epsilon)
            {
                candidate.metric_damage =
                    0.5 *
                    std::log(
                        rayleigh);
            }

            // --------------------------------------------------------
            // Set-cover relation.
            //
            // Candidate face covers obstacle k iff that obstacle lies
            // strictly outside the candidate halfspace.
            // --------------------------------------------------------
            for (int pointId = 0;
                 pointId <
                     obstacleCount;
                 ++pointId)
            {
                const double violation =
                    normal.dot(
                        localObstacles[
                            pointId]) -
                    threshold;

                ++localDiagnostics.obstacle_face_tests; 

                if (violation >
                    epsilon)
                {
                    candidate
                        .covered_obstacles
                        .push_back(
                            pointId);
                }
            }

            if (std::find(
                    candidate
                        .covered_obstacles
                        .begin(),
                    candidate
                        .covered_obstacles
                        .end(),
                    obstacleId) ==
                candidate
                    .covered_obstacles
                    .end())
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            candidates.emplace_back(
                std::move(
                    candidate));
        }

        localDiagnostics.generated_candidate_count =
            static_cast<int>(
                candidates.size());
            
        localDiagnostics.candidate_count =
            localDiagnostics.generated_candidate_count;

        // ------------------------------------------------------------
        // Batch greedy set cover.
        //
        // Lexicographic choice:
        //
        //   1. maximize number of newly excluded obstacles;
        //   2. minimize CSGN face damage psi;
        //   3. maximize seed-obstacle clearance.
        //
        // This is an approximate minimum-face solver.
        // It can later be replaced by branch-and-bound without changing
        // candidate geometry.
        // ------------------------------------------------------------
        std::vector<unsigned char>
            unresolved(
                obstacleCount,
                1);

        selected.assign(
            candidates.size(),
            0);

        int unresolvedCount =
            obstacleCount;

        while (unresolvedCount >
               0)
        {
            int bestCandidate =
                -1;

            int bestGain =
                0;

            double bestDamage =
                std::numeric_limits<double>::
                    infinity();

            double bestClearance =
                -std::numeric_limits<double>::
                    infinity();

            for (int candidateId = 0;
                 candidateId <
                     static_cast<int>(
                         candidates.size());
                 ++candidateId)
            {
                if (selected[
                        candidateId])
                {
                    continue;
                }

                int gain =
                    0;

                for (const int obstacleId :
                     candidates[
                         candidateId]
                         .covered_obstacles)
                {
                    if (unresolved[
                            obstacleId])
                    {
                        ++gain;
                    }
                }

                if (gain <=
                    0)
                {
                    continue;
                }

                const double damage =
                    candidates[
                        candidateId]
                        .metric_damage;

                const double clearance =
                    candidates[
                        candidateId]
                        .seed_obstacle_clearance;

                const bool prefer =
                    gain >
                        bestGain ||

                    (gain ==
                         bestGain &&
                     damage <
                         bestDamage -
                             epsilon) ||

                    (gain ==
                         bestGain &&
                     std::abs(
                         damage -
                         bestDamage) <=
                         epsilon &&
                     clearance >
                         bestClearance);

                if (prefer)
                {
                    bestCandidate =
                        candidateId;

                    bestGain =
                        gain;

                    bestDamage =
                        damage;

                    bestClearance =
                        clearance;
                }
            }

            if (bestCandidate <
                    0 ||
                bestGain <=
                    0)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            selected[
                bestCandidate] =
                1;

            selectedCandidateIds
                .push_back(
                    bestCandidate);

            for (const int obstacleId :
                 candidates[
                     bestCandidate]
                     .covered_obstacles)
            {
                if (unresolved[
                        obstacleId])
                {
                    unresolved[
                        obstacleId] =
                        0;

                    --unresolvedCount;
                }
            }
        }

        localDiagnostics
            .greedy_obstacle_face_count =
            static_cast<int>(
                selectedCandidateIds.size());
    }
    else
    {
        std::vector<unsigned char>
            unresolved(
                obstacleCount,
                1);

        int unresolvedCount =
            obstacleCount;

        while (unresolvedCount > 0)
        {
            int witnessId =
                -1;

            double witnessMetricDistanceSquared =
                std::numeric_limits<double>::
                    infinity();

            double witnessInterpolation =
                0.0;

            // --------------------------------------------------------
            // Pricing / witness step:
            //
            // Find the closest unresolved obstacle to the protected
            // segment in the CSGN metric W.
            // --------------------------------------------------------
            for (int obstacleId = 0;
                 obstacleId < obstacleCount;
                 ++obstacleId)
            {
                if (!unresolved[
                        obstacleId])
                {
                    continue;
                }
                const Eigen::Vector3d &obstacle =
                    localObstacles[
                        obstacleId];

                double interpolation =
                    0.0;

                if (segmentMetricNormSquared >
                    epsilon * epsilon)
                {
                    interpolation =
                        segment.dot(
                            inverseUtility *
                            (obstacle - a)) /
                        segmentMetricNormSquared;

                    interpolation =
                        std::max(
                            0.0,
                            std::min(
                                1.0,
                                interpolation));
                }

                const Eigen::Vector3d projection =
                    a +
                    interpolation *
                        segment;

                const Eigen::Vector3d residual =
                    obstacle -
                    projection;

                const double metricDistanceSquared =
                    residual.dot(
                        inverseUtility *
                        residual);

                ++localDiagnostics
                      .witness_distance_tests;

                if (!std::isfinite(
                        metricDistanceSquared))
                {
                    if (diagnostics != nullptr)
                    {
                        *diagnostics =
                            localDiagnostics;
                    }

                    return false;
                }

                if (metricDistanceSquared <
                    witnessMetricDistanceSquared)
                {
                    witnessMetricDistanceSquared =
                        metricDistanceSquared;

                    witnessId =
                        obstacleId;

                    witnessInterpolation =
                        interpolation;
                }
            }

            if (witnessId < 0)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            const Eigen::Vector3d &witness =
                localObstacles[
                    witnessId];

            const Eigen::Vector3d projection =
                a +
                witnessInterpolation *
                    segment;

            Eigen::Vector3d normal =
                inverseUtility *
                (witness -
                 projection);

            const double normalNorm =
                normal.norm();

            if (!std::isfinite(
                    normalNorm) ||
                normalNorm <=
                    epsilon)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            normal /=
                normalNorm;

            const double protectedSupport =
                std::max(
                    normal.dot(a),
                    normal.dot(b)) +
                overlapRadius;

            const double obstacleSupport =
                normal.dot(
                    witness);

            const double supportGap =
                obstacleSupport -
                protectedSupport;

            if (!std::isfinite(
                    supportGap) ||
                supportGap <=
                    2.0 * epsilon)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            const double threshold =
                0.5 *
                (protectedSupport +
                 obstacleSupport);

            CandidateFace candidate;

            candidate.source_obstacle =
                witnessId;

            candidate.plane.head<3>() =
                normal;

            candidate.plane(3) =
                -threshold;

            candidate.seed_obstacle_clearance =
                0.5 *
                supportGap;

            const double rayleigh =
                normal.dot(
                    utility *
                    normal);

            if (std::isfinite(rayleigh) &&
                rayleigh > epsilon)
            {
                candidate.metric_damage =
                    0.5 *
                    std::log(
                        rayleigh);
            }

            // --------------------------------------------------------
            // One generated face may eliminate MANY unresolved
            // obstacles.  This is the batch-elimination step.
            // --------------------------------------------------------
            for (int obstacleId = 0;
                 obstacleId < obstacleCount;
                 ++obstacleId)
            {
                if (!unresolved[
                        obstacleId])
                {
                    continue;
                }

                ++localDiagnostics
                      .obstacle_face_tests;

                const double violation =
                    normal.dot(
                        localObstacles[
                            obstacleId]) -
                    threshold;

                if (violation >
                    epsilon)
                {
                    candidate
                        .covered_obstacles
                        .push_back(
                            obstacleId);
                }
            }

            if (candidate
                    .covered_obstacles
                    .empty())
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            bool witnessCovered =
                false;

            for (const int obstacleId :
                 candidate
                     .covered_obstacles)
            {
                if (obstacleId ==
                    witnessId)
                {
                    witnessCovered =
                        true;
                }

                if (unresolved[
                        obstacleId])
                {
                    unresolved[
                        obstacleId] =
                        0;

                    --unresolvedCount;
                }
            }

            if (!witnessCovered)
            {
                if (diagnostics != nullptr)
                {
                    *diagnostics =
                        localDiagnostics;
                }

                return false;
            }

            candidates.push_back(
                std::move(
                    candidate));

            selectedCandidateIds
                .push_back(
                    static_cast<int>(
                        candidates.size()) -
                    1);

            ++localDiagnostics
                  .generated_candidate_count;

            ++localDiagnostics
                  .active_witness_rounds;
        }

        // ============================================================
        // Rebuild COMPLETE coverage relations for active candidates.
        //
        // During active generation, candidate.covered_obstacles contains
        // only obstacles that were unresolved at the moment the face was
        // generated.  Those lists are therefore nearly disjoint and are
        // insufficient for the common reverse-delete stage.
        //
        // Recompute every selected face against the complete local cloud.
        // Complexity is O(F * N), with F equal to the number of generated
        // active faces.
        // ============================================================
        for (auto &candidate :
             candidates)
        {
            candidate.covered_obstacles.clear();
        
            const Eigen::Vector3d normal =
                candidate.plane.head<3>();
        
            const double threshold =
                -candidate.plane(3);
        
            for (int obstacleId = 0;
                 obstacleId < obstacleCount;
                 ++obstacleId)
            {
                ++localDiagnostics
                      .obstacle_face_tests;
            
                const double violation =
                    normal.dot(
                        localObstacles[
                            obstacleId]) -
                    threshold;
                        
                if (violation >
                    epsilon)
                {
                    candidate
                        .covered_obstacles
                        .push_back(
                            obstacleId);
                }
            }
        }

        localDiagnostics
            .candidate_count =
            localDiagnostics
                .generated_candidate_count;

        localDiagnostics
            .greedy_obstacle_face_count =
            static_cast<int>(
                selectedCandidateIds.size());

        selected.assign(
            candidates.size(),
            1);
    }

    // ------------------------------------------------------------
    // Reverse-delete redundancy pruning.
    //
    // Greedy set cover may contain a face that becomes unnecessary
    // after subsequent choices. Test high-damage faces first.
    // ------------------------------------------------------------
    std::vector<int>
        coverCount(
            obstacleCount,
            0);

    for (const int candidateId :
         selectedCandidateIds)
    {
        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            ++coverCount[
                obstacleId];
        }
    }

    std::vector<int>
        removalOrder =
            selectedCandidateIds;

    std::sort(
        removalOrder.begin(),
        removalOrder.end(),
        [&](const int lhs,
            const int rhs)
        {
            const double lhsDamage =
                candidates[lhs]
                    .metric_damage;

            const double rhsDamage =
                candidates[rhs]
                    .metric_damage;

            if (std::abs(
                    lhsDamage -
                    rhsDamage) >
                epsilon)
            {
                return
                    lhsDamage >
                    rhsDamage;
            }

            return
                candidates[lhs]
                    .covered_obstacles
                    .size() <
                candidates[rhs]
                    .covered_obstacles
                    .size();
        });

    for (const int candidateId :
         removalOrder)
    {
        if (!selected[
                candidateId])
        {
            continue;
        }

        bool removable =
            true;

        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            if (coverCount[
                    obstacleId] <=
                1)
            {
                removable =
                    false;

                break;
            }
        }

        if (!removable)
        {
            continue;
        }

        selected[
            candidateId] =
            0;

        ++localDiagnostics
              .redundancy_removed;

        for (const int obstacleId :
             candidates[
                 candidateId]
                 .covered_obstacles)
        {
            --coverCount[
                obstacleId];
        }
    }

    int selectedObstacleFaceCount =
        0;

    for (const unsigned char flag :
         selected)
    {
        selectedObstacleFaceCount +=
            flag
                ? 1
                : 0;
    }

    localDiagnostics
        .selected_obstacle_face_count =
        selectedObstacleFaceCount;

    // ------------------------------------------------------------
    // Assemble final H-polytope.
    // ------------------------------------------------------------
    hPoly.resize(
        fixedH.rows() +
            selectedObstacleFaceCount,
        4);

    hPoly.topRows(
        fixedH.rows()) =
        fixedH;

    int outputRow =
        fixedH.rows();

    double damageSum =
        0.0;

    double minDamage =
        std::numeric_limits<double>::
            infinity();

    double maxDamage =
        -std::numeric_limits<double>::
            infinity();

    for (int candidateId = 0;
         candidateId <
             static_cast<int>(
                 candidates.size());
         ++candidateId)
    {
        if (!selected[
                candidateId])
        {
            continue;
        }

        hPoly.row(
            outputRow++) =
            candidates[
                candidateId]
                .plane
                .transpose();

        const double damage =
            candidates[
                candidateId]
                .metric_damage;

        damageSum +=
            damage;

        minDamage =
            std::min(
                minDamage,
                damage);

        maxDamage =
            std::max(
                maxDamage,
                damage);
    }

    localDiagnostics.total_face_count =
        static_cast<int>(
            hPoly.rows());

    if (selectedObstacleFaceCount >
        0)
    {
        localDiagnostics.mean_metric_damage =
            damageSum /
            static_cast<double>(
                selectedObstacleFaceCount);

        localDiagnostics.min_metric_damage =
            minDamage;

        localDiagnostics.max_metric_damage =
            maxDamage;
    }

    // ------------------------------------------------------------
    // Explicit overlap / seed-capsule verification.
    // ------------------------------------------------------------
    for (int faceId = 0;
         faceId <
             hPoly.rows();
         ++faceId)
    {
        const Eigen::Vector3d normal =
            hPoly.block<1, 3>(
                    faceId,
                    0)
                .transpose();

        const double normalNorm =
            normal.norm();

        if (!std::isfinite(
                normalNorm) ||
            normalNorm <=
                epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }

        const double protectedValue =
            std::max(
                normal.dot(a),
                normal.dot(b)) +
            overlapRadius *
                normalNorm +
            hPoly(
                faceId,
                3);

        if (protectedValue >
            epsilon)
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    localDiagnostics.overlap_guaranteed =
        true;

    // ------------------------------------------------------------
    // Full local-cloud safety verification.
    //
    // Every obstacle inside the trajectory-relevant domain must lie
    // outside at least one final halfspace.
    // ------------------------------------------------------------
    for (int obstacleId = 0;
         obstacleId <
             obstacleCount;
         ++obstacleId)
    {
        const Eigen::Vector3d &point =
            localObstacles[
                obstacleId];

        const Eigen::Vector4d pointH(
            point(0),
            point(1),
            point(2),
            1.0);

        const double maxViolation =
            (hPoly *
             pointH)
                .maxCoeff();

        if (!(maxViolation >
              epsilon))
        {
            if (diagnostics != nullptr)
            {
                *diagnostics =
                    localDiagnostics;
            }

            return false;
        }
    }

    localDiagnostics.safety_verified =
        true;

    if (diagnostics != nullptr)
    {
        *diagnostics =
            localDiagnostics;
    }

    return true;
}

} // namespace traj_relevant

#endif
