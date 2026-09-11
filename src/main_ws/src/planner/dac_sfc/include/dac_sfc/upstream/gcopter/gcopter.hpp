/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <string>

namespace gcopter
{

    class GCOPTER_PolytopeSFC
    {
    public:
        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::MatrixX4d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

        struct CorridorDiagnostics
        {
            int constrainedPieceCount = 0;
        
            double penaltyCost =
                0.0;
        
            double maxViolationM =
                0.0;

            int maxViolationPiece =
                -1;

            int maxViolationCorridor =
                -1;

            int maxViolationFace =
                -1;

            int maxViolationSample =
                -1;

            double maxViolationTime =
                0.0;

            Eigen::Vector3d maxViolationPosition =
                Eigen::Vector3d::Zero();
        
            // Minimum normalized distance from sampled trajectory
            // positions to their assigned corridor faces.
            //
            // Positive  : inside the corridor.
            // Zero      : touching a face.
            // Negative  : outside / violated.
            double minSlackM =
                INFINITY;
        };

        enum class DeformationMetricObjective
        {
            ENERGY_ONLY = 0,
            DYNAMICS_ONLY = 1,
            FULL = 2
        };

        struct DeformationMetric
        {
            // Local curvature of the full GCOPTER objective with respect to
            // a Cartesian deformation of one trajectory piece.
            Eigen::Matrix3d stiffness =
                Eigen::Matrix3d::Identity();

            // Inverse-curvature metric. Large value along a direction means
            // that the optimized trajectory can deform more easily there.
            Eigen::Matrix3d utility =
                Eigen::Matrix3d::Identity();

            // Eigenvalues before and after positive regularization.
            Eigen::Vector3d rawStiffnessEigenvalues =
                Eigen::Vector3d::Ones();

            Eigen::Vector3d regularizedStiffnessEigenvalues =
                Eigen::Vector3d::Ones();

            // Principal eigenvector of utility.
            Eigen::Vector3d principalDirection =
                Eigen::Vector3d::UnitX();

            Eigen::Matrix3d scalarStiffness =
                Eigen::Matrix3d::Identity();

            double gradientScalarRelativeError =
                INFINITY;
            
            // lambda_max(S) / lambda_min(S).
            double anisotropy = 1.0;
            double rawAnisotropy = 1.0;
            double symmetryError = INFINITY;

            bool valid = false;

            std::string failureReason = "none";

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef std::vector<
            DeformationMetric,
            Eigen::aligned_allocator<DeformationMetric>>
            DeformationMetrics;

        struct GaussNewtonDeformationMetric
        {
            // Unweighted J^T J, kept for comparison.
            Eigen::Matrix3d rawGram =
                Eigen::Matrix3d::Zero();

            // gram below will become the proximity-weighted Gram matrix.
            Eigen::Matrix3d gram =
                Eigen::Matrix3d::Zero();

            // Nominal constraint proximity diagnostics.
            // Order:
            //   [velocity, body rate, tilt, thrust]
            Eigen::Vector4d meanProximityRatios =
                Eigen::Vector4d::Zero();

            Eigen::Vector4d maxProximityRatios =
                Eigen::Vector4d::Zero();

            Eigen::Vector4d meanProximityWeights =
                Eigen::Vector4d::Zero();

            double proximityPowerUsed = 0.0;

            double rawAnisotropy = 1.0;
            
            // ------------------------------------------------------------
            // Per-feature Gauss-Newton contributions:
            //
            //     G = G_v + G_omega + G_theta + G_thrust
            //
            // These matrices are diagnostic for understanding which
            // physical quantity dominates the deformation metric.
            // ------------------------------------------------------------
            Eigen::Matrix3d velocityGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d bodyRateGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d tiltGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d thrustGram =
                Eigen::Matrix3d::Zero();
        
            // Positive-definite deformation stiffness:
            //
            //     K = J^T J + lambda I.
            Eigen::Matrix3d stiffness =
                Eigen::Matrix3d::Identity();
        
            // Determinant-normalized inverse stiffness.
            //
            // Large value means easier trajectory deformation.
            Eigen::Matrix3d utility =
                Eigen::Matrix3d::Identity();
        
            Eigen::Vector3d stiffnessEigenvalues =
                Eigen::Vector3d::Ones();
        
            Eigen::Vector3d utilityEigenvalues =
                Eigen::Vector3d::Ones();
        
            Eigen::Vector3d principalDirection =
                Eigen::Vector3d::UnitX();

            // ------------------------------------------------------------
            // Spectrum-compressed utility used by the downstream corridor
            // generator.
            //
            // `utility` remains the natural determinant-normalized inverse
            // CSGN metric.
            //
            // `corridorUtility` preserves its eigenvectors and eigenvalue
            // ordering, but bounds the anisotropy.
            // ------------------------------------------------------------
            Eigen::Matrix3d corridorUtility =
                Eigen::Matrix3d::Identity();

            Eigen::Vector3d corridorUtilityEigenvalues =
                Eigen::Vector3d::Ones();

            double corridorAnisotropy = 1.0;

            double spectrumCompressionAlpha = 1.0;

            double maxCorridorAnisotropyUsed = 1.0;
        
            double displacementStepUsed = 0.0;
        
            double dampingUsed = 0.0;
        
            double jacobianFrobeniusNorm = 0.0;
        
            // lambda_max(S) / lambda_min(S)
            double anisotropy = 1.0;
        
            // lambda_max(S) / lambda_2(S).
            // If this is close to 1, the principal direction
            // should NOT be regarded as unique.
            double principalGap = 1.0;

            double velocityTraceFraction = 0.0;
            double bodyRateTraceFraction = 0.0;
            double tiltTraceFraction = 0.0;
            double thrustTraceFraction = 0.0;

            // Directionality of each contribution:
            //
            //     ||G_q - tr(G_q)/3 I||_F / tr(G_q)
            //
            // 0 means perfectly isotropic.
            // Larger values mean stronger directional structure.
            double velocityDirectionality = 0.0;
            double bodyRateDirectionality = 0.0;
            double tiltDirectionality = 0.0;
            double thrustDirectionality = 0.0;

            // Should be approximately zero.  Used to verify that the
            // decomposition exactly reconstructs the full J^T J.
            double decompositionRelativeError = INFINITY;
        
            bool valid = false;
        
            std::string failureReason = "none";
        
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef std::vector<
            GaussNewtonDeformationMetric,
            Eigen::aligned_allocator<
                GaussNewtonDeformationMetric>>
            GaussNewtonDeformationMetrics;

    private:
        minco::MINCO_S3NU minco;
        flatness::FlatnessMap flatmap;

        double rho;
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;

        PolyhedraV vPolytopes;
        PolyhedraH hPolytopes;
        Eigen::Matrix3Xd shortPath;

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;
        Eigen::VectorXi hPolyIdx;

        int polyN;
        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;

        Eigen::VectorXd penaltyWt;

        // Baseline penalty weights supplied by setup().
        // Continuation scales only the corridor-position term
        // relative to this immutable reference.
        Eigen::VectorXd basePenaltyWt;

        Eigen::VectorXd physicalPm;
        double allocSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::VectorXd times;
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

        Eigen::VectorXd optimizedX;
        Eigen::Matrix3Xd optimizedPoints;
        Eigen::VectorXd optimizedTimes;
        double optimizedCost = INFINITY;
        bool optimizedStateValid = false;

        CorridorDiagnostics initialCorridorDiagnostics;
        CorridorDiagnostics finalCorridorDiagnostics;

    private:
        inline CorridorDiagnostics evaluateCorridorDiagnostics(
            const Trajectory<5> &trajectory) const
        {
            CorridorDiagnostics diagnostics;
            const int sampleCount = std::max(integralRes, 1);
            const int count = std::min(
                trajectory.getPieceNum(), static_cast<int>(hPolyIdx.size()));
            for (int pieceId = 0; pieceId < count; ++pieceId)
            {
                const int corridorId = hPolyIdx(pieceId);
                if (corridorId < 0 || corridorId >= static_cast<int>(hPolytopes.size()))
                {
                    continue;
                }
                ++diagnostics.constrainedPieceCount;
                const Piece<5> &piece = trajectory[pieceId];
                const double step = piece.getDuration() /
                                    static_cast<double>(sampleCount);
                for (int sampleId = 0; sampleId <= sampleCount; ++sampleId)
                {
                    const Eigen::Vector3d position =
                        piece.getPos(step * static_cast<double>(sampleId));
                    double samplePenalty = 0.0;
                    for (int faceId = 0;
                         faceId < hPolytopes[corridorId].rows(); ++faceId)
                    {
                        const Eigen::Vector3d normal =
                            hPolytopes[corridorId].row(faceId).head<3>().transpose();
                        const double normalNorm = normal.norm();
                        if (normalNorm < 1e-9)
                        {
                            continue;
                        }

                        const double violation =
                            normal.dot(position) +
                            hPolytopes[corridorId](faceId, 3);

                        const double normalizedViolation =
                            violation / normalNorm;

                        if (normalizedViolation >
                            diagnostics.maxViolationM)
                        {
                            diagnostics.maxViolationM =
                                normalizedViolation;

                            diagnostics.maxViolationPiece =
                                pieceId;

                            diagnostics.maxViolationCorridor =
                                corridorId;

                            diagnostics.maxViolationFace =
                                faceId;

                            diagnostics.maxViolationSample =
                                sampleId;

                            diagnostics.maxViolationTime =
                                step *
                                static_cast<double>(
                                    sampleId);

                            diagnostics.maxViolationPosition =
                                position;
                        }
                            
                        diagnostics.minSlackM =
                            std::min(
                                diagnostics.minSlackM,
                                -normalizedViolation);
                                
                        double smoothedPenalty = 0.0;
                        double smoothedPenaltyDerivative = 0.0;
                        if (smoothedL1(violation, smoothEps, smoothedPenalty,
                                       smoothedPenaltyDerivative))
                        {
                            samplePenalty += penaltyWt(0) * smoothedPenalty;
                        }
                    }
                    const double quadratureWeight =
                        (sampleId == 0 || sampleId == sampleCount) ? 0.5 : 1.0;
                    diagnostics.penaltyCost +=
                        quadratureWeight * step * samplePenalty;
                }
            }
            if (!std::isfinite(
                    diagnostics.minSlackM))
            {
                diagnostics.minSlackM =
                    0.0;
            }
            return diagnostics;
        }

        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_PolytopeSFC::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const Eigen::VectorXi &hIdx,
                                                   const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   flatness::FlatnessMap &flatMap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC)
        {
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double thetaMax = magnitudeBounds(2);
            const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
            const double thrustSqrRadi = thrustRadi * thrustRadi;

            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightOmg = penaltyWeights(2);
            const double weightTheta = penaltyWeights(3);
            const double weightThrust = penaltyWeights(4);

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
            double thr, cos_theta;
            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            double gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradOmg;

            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int K, L;
            double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
            double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
            double node, pena;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

                    violaVel = vel.squaredNorm() - velSqrMax;
                    violaOmg = omg.squaredNorm() - omgSqrMax;
                    cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                    violaTheta = acos(cos_theta) - thetaMax;
                    violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    pena = 0.0;

                    L = hIdx(i);
                    K = hPolys[L].rows();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                    }

                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                        pena += weightOmg * violaOmgPena;
                    }

                    if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
                    {
                        gradQuat += weightTheta * violaThetaPenaD /
                                    sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                                    Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                        pena += weightTheta * violaThetaPena;
                    }

                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
                    {
                        gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
                        pena += weightThrust * violaThrustPena;
                    }

                    flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                     totalGradPsi, totalGradPsiD);

                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                    beta1 * totalGradVel.transpose() +
                                                    beta2 * totalGradAcc.transpose() +
                                                    beta3 * totalGradJer.transpose()) *
                                                   node * step;
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        inline double evaluateObjectiveAt(const Eigen::VectorXd &x,
                                          Eigen::VectorXd &gradient)
        {
            if (x.size() != temporalDim + spatialDim)
            {
                gradient.resize(0);
                return INFINITY;
            }

            gradient.resize(x.size());
            gradient.setZero();

            return costFunctional(this, x, gradient);
        }

        inline double evaluateObjectiveAtPoints(
            const Eigen::Matrix3Xd &testPoints,
            Eigen::Matrix3Xd &gradientByPoints,
            const DeformationMetricObjective objectiveMode)
        {
            if (!optimizedStateValid)
            {
                gradientByPoints.resize(3, 0);
                return INFINITY;
            }
        
            if (testPoints.rows() != 3 ||
                testPoints.cols() != pieceN - 1)
            {
                gradientByPoints.resize(3, 0);
                return INFINITY;
            }
        
            if (optimizedTimes.size() != pieceN)
            {
                gradientByPoints.resize(3, 0);
                return INFINITY;
            }
        
            // ----------------------------------------------------
            // MINCO is evaluated directly in Cartesian waypoint
            // coordinates with the optimized durations fixed.
            // ----------------------------------------------------
            minco.setParameters(
                testPoints,
                optimizedTimes);
            
            double cost = 0.0;
            
            minco.getEnergy(cost);
            
            minco.getEnergyPartialGradByCoeffs(
                partialGradByCoeffs);
            
            minco.getEnergyPartialGradByTimes(
                partialGradByTimes);
            
            // ----------------------------------------------------
            // ENERGY_ONLY:
            //
            //     J_metric = J_MINCO
            //
            // No physical/corridor penalty is attached.
            // ----------------------------------------------------
            if (objectiveMode !=
                DeformationMetricObjective::ENERGY_ONLY)
            {
                Eigen::VectorXd metricPenaltyWeights =
                    penaltyWt;
            
                // -----------------------------------------------
                // DYNAMICS_ONLY:
                //
                // remove the position/corridor penalty but keep
                //
                // velocity
                // body rate
                // tilt
                // thrust
                //
                // penalties.
                // -----------------------------------------------
                if (objectiveMode ==
                    DeformationMetricObjective::DYNAMICS_ONLY)
                {
                    if (metricPenaltyWeights.size() > 0)
                    {
                        metricPenaltyWeights(0) = 0.0;
                    }
                }
            
                attachPenaltyFunctional(
                    optimizedTimes,
                    minco.getCoeffs(),
                    hPolyIdx,
                    hPolytopes,
                    smoothEps,
                    integralRes,
                    magnitudeBd,
                    metricPenaltyWeights,
                    flatmap,
                    cost,
                    partialGradByTimes,
                    partialGradByCoeffs);
            }
        
            minco.propogateGrad(
                partialGradByCoeffs,
                partialGradByTimes,
                gradByPoints,
                gradByTimes);
            
            // T is fixed during deformation analysis, therefore
            // rho * sum(T) is only a constant and has no influence
            // on the Cartesian Hessian. Keeping it here preserves
            // the numerical objective value.
            cost +=
                rho *
                optimizedTimes.sum();
            
            gradientByPoints =
                gradByPoints;
            
            if (!std::isfinite(cost) ||
                !gradientByPoints.allFinite())
            {
                return INFINITY;
            }
        
            return cost;
        }
    
        inline Eigen::Vector3d projectPointGradientToPiece(
            const int pieceId,
            const Eigen::Matrix3Xd &pointGradient) const
        {
            Eigen::Vector3d projected =
                Eigen::Vector3d::Zero();
        
            int movablePointCount = 0;
        
            if (pieceId > 0)
            {
                ++movablePointCount;
            }
        
            if (pieceId < pieceN - 1)
            {
                ++movablePointCount;
            }
        
            if (movablePointCount <= 0)
            {
                return projected;
            }
        
            // Normalized deformation basis:
            //
            // D^T D = I_3
            //
            // for both boundary and interior pieces.
            const double weight =
                1.0 /
                std::sqrt(
                    static_cast<double>(
                        movablePointCount));
                    
            if (pieceId > 0)
            {
                projected +=
                    weight *
                    pointGradient.col(pieceId - 1);
            }
        
            if (pieceId < pieceN - 1)
            {
                projected +=
                    weight *
                    pointGradient.col(pieceId);
            }
        
            return projected;
        }

        inline bool buildPerturbedPointsForPiece(
            const int pieceId,
            const int axis,
            const double displacement,
            Eigen::Matrix3Xd &plusPoints,
            Eigen::Matrix3Xd &minusPoints) const
        {
            if (!optimizedStateValid ||
                pieceId < 0 ||
                pieceId >= pieceN ||
                axis < 0 ||
                axis >= 3 ||
                displacement <= 0.0)
            {
                return false;
            }
        
            plusPoints =
                optimizedPoints;
        
            minusPoints =
                optimizedPoints;
        
            int movablePointCount = 0;
        
            if (pieceId > 0)
            {
                ++movablePointCount;
            }
        
            if (pieceId < pieceN - 1)
            {
                ++movablePointCount;
            }
        
            if (movablePointCount <= 0)
            {
                return false;
            }
        
            const double weight =
                1.0 /
                std::sqrt(
                    static_cast<double>(
                        movablePointCount));
                    
            Eigen::Vector3d delta =
                Eigen::Vector3d::Zero();
                    
            delta(axis) =
                weight * displacement;
                    
            if (pieceId > 0)
            {
                plusPoints.col(pieceId - 1) +=
                    delta;
            
                minusPoints.col(pieceId - 1) -=
                    delta;
            }
        
            if (pieceId < pieceN - 1)
            {
                plusPoints.col(pieceId) +=
                    delta;
            
                minusPoints.col(pieceId) -=
                    delta;
            }
        
            return plusPoints.allFinite() &&
                   minusPoints.allFinite();
        }

        inline bool buildDeformedPointsForPiece(
            const int pieceId,
            const Eigen::Vector3d &deformation,
            Eigen::Matrix3Xd &deformedPoints) const
        {
            if (!optimizedStateValid ||
                pieceId < 0 ||
                pieceId >= pieceN ||
                !deformation.allFinite())
            {
                return false;
            }
        
            deformedPoints =
                optimizedPoints;
        
            int movablePointCount = 0;
        
            if (pieceId > 0)
            {
                ++movablePointCount;
            }
        
            if (pieceId < pieceN - 1)
            {
                ++movablePointCount;
            }
        
            if (movablePointCount <= 0)
            {
                return false;
            }
        
            // Normalized piece deformation basis:
            //
            //     D_i^T D_i = I_3
            //
            // Interior pieces contain two movable waypoints, so each
            // waypoint receives deformation / sqrt(2).
            const double weight =
                1.0 /
                std::sqrt(
                    static_cast<double>(
                        movablePointCount));
                    
            if (pieceId > 0)
            {
                deformedPoints.col(pieceId - 1) +=
                    weight * deformation;
            }
        
            if (pieceId < pieceN - 1)
            {
                deformedPoints.col(pieceId) +=
                    weight * deformation;
            }
        
            return deformedPoints.allFinite();
        }

        inline bool evaluateDynamicsFeatureVector(
            const Eigen::Matrix3Xd &testPoints,
            Eigen::VectorXd &featureVector)
        {
            featureVector.resize(0);
        
            if (!optimizedStateValid)
            {
                return false;
            }
        
            if (testPoints.rows() != 3 ||
                testPoints.cols() != pieceN - 1)
            {
                return false;
            }
        
            if (optimizedTimes.size() != pieceN ||
                pieceN <= 0 ||
                integralRes <= 0)
            {
                return false;
            }
        
            if (magnitudeBd.size() < 5)
            {
                return false;
            }
        
            // ------------------------------------------------------------
            // Dimensionless dynamics scales.
            // ------------------------------------------------------------
            const double velocityScale =
                std::max(
                    std::abs(magnitudeBd(0)),
                    1.0e-6);
                
            const double bodyRateScale =
                std::max(
                    std::abs(magnitudeBd(1)),
                    1.0e-6);
                
            const double thetaMax =
                std::max(
                    std::abs(magnitudeBd(2)),
                    1.0e-6);
                
            const double tiltScale =
                std::max(
                    std::sin(0.5 * thetaMax),
                    1.0e-6);
                
            const double thrustMean =
                0.5 *
                (magnitudeBd(3) +
                 magnitudeBd(4));
                
            const double thrustRadius =
                std::max(
                    0.5 *
                        std::abs(
                            magnitudeBd(4) -
                            magnitudeBd(3)),
                    1.0e-6);
                        
            // ------------------------------------------------------------
            // Reconstruct the MINCO trajectory for the perturbed Cartesian
            // inner waypoints. Durations remain fixed at T*.
            // ------------------------------------------------------------
            minco.setParameters(
                testPoints,
                optimizedTimes);
            
            Trajectory<5> metricTrajectory;
            
            minco.getTrajectory(
                metricTrajectory);
            
            if (metricTrajectory.getPieceNum() != pieceN)
            {
                return false;
            }
        
            // Each quadrature node contributes:
            //
            //   3 normalized velocity components
            // + 3 normalized body-rate components
            // + 2 normalized tilt quaternion components
            // + 1 normalized thrust component
            //
            // = 9 features.
            constexpr int featurePerNode = 9;
        
            const int nodeCount =
                pieceN *
                (integralRes + 1);
        
            featureVector.resize(
                featurePerNode *
                nodeCount);
            
            int offset = 0;
            
            for (int pieceId = 0;
                 pieceId < pieceN;
                 ++pieceId)
            {
                const double duration =
                    optimizedTimes(pieceId);
            
                if (!std::isfinite(duration) ||
                    duration <= 0.0)
                {
                    featureVector.resize(0);
                    return false;
                }
            
                const double dt =
                    duration /
                    static_cast<double>(
                        integralRes);
                    
                const Piece<5> &piece =
                    metricTrajectory[pieceId];
                    
                for (int sampleId = 0;
                     sampleId <= integralRes;
                     ++sampleId)
                {
                    const double alpha =
                        static_cast<double>(sampleId) /
                        static_cast<double>(integralRes);
                
                    const double localTime =
                        alpha * duration;
                
                    const double trapezoidWeight =
                        (sampleId == 0 ||
                         sampleId == integralRes)
                            ? 0.5
                            : 1.0;
                        
                    // sqrt(weight * dt) makes J^T J approximate a
                    // time integral of squared feature sensitivities.
                    const double featureWeight =
                        std::sqrt(
                            trapezoidWeight *
                            dt);
                        
                    const Eigen::Vector3d vel =
                        piece.getVel(localTime);
                        
                    const Eigen::Vector3d acc =
                        piece.getAcc(localTime);
                        
                    const Eigen::Vector3d jer =
                        piece.getJer(localTime);
                        
                    double thrust = 0.0;
                        
                    Eigen::Vector4d quat =
                        Eigen::Vector4d::Zero();
                        
                    Eigen::Vector3d bodyRate =
                        Eigen::Vector3d::Zero();
                        
                    flatmap.forward(
                        vel,
                        acc,
                        jer,
                        0.0,
                        0.0,
                        thrust,
                        quat,
                        bodyRate);
                    
                    if (!vel.allFinite() ||
                        !acc.allFinite() ||
                        !jer.allFinite() ||
                        !std::isfinite(thrust) ||
                        !quat.allFinite() ||
                        !bodyRate.allFinite())
                    {
                        featureVector.resize(0);
                        return false;
                    }
                
                    featureVector.segment<3>(
                        offset) =
                        featureWeight *
                        vel /
                        velocityScale;
                    
                    featureVector.segment<3>(
                        offset + 3) =
                        featureWeight *
                        bodyRate /
                        bodyRateScale;
                    
                    featureVector(offset + 6) =
                        featureWeight *
                        quat(1) /
                        tiltScale;
                    
                    featureVector(offset + 7) =
                        featureWeight *
                        quat(2) /
                        tiltScale;
                    
                    featureVector(offset + 8) =
                        featureWeight *
                        (thrust - thrustMean) /
                        thrustRadius;
                    
                    offset +=
                        featurePerNode;
                }
            }
        
            return featureVector.allFinite();
        }

        inline bool evaluateConstraintUtilizationFeatureVector(
            const Eigen::Matrix3Xd &testPoints,
            Eigen::VectorXd &featureVector)
        {
            featureVector.resize(0);
        
            if (!optimizedStateValid ||
                pieceN <= 0 ||
                integralRes <= 0)
            {
                return false;
            }
        
            if (testPoints.rows() != 3 ||
                testPoints.cols() != pieceN - 1 ||
                optimizedTimes.size() != pieceN ||
                magnitudeBd.size() < 5)
            {
                return false;
            }
        
            const double velocityScale =
                std::max(
                    std::abs(magnitudeBd(0)),
                    1.0e-6);
                
            const double bodyRateScale =
                std::max(
                    std::abs(magnitudeBd(1)),
                    1.0e-6);
                
            const double thetaMax =
                std::max(
                    std::abs(magnitudeBd(2)),
                    1.0e-6);
                
            const double tiltScale =
                std::max(
                    std::sin(0.5 * thetaMax),
                    1.0e-6);
                
            const double thrustMean =
                0.5 *
                (magnitudeBd(3) +
                 magnitudeBd(4));
                
            const double thrustRadius =
                std::max(
                    0.5 *
                        std::abs(
                            magnitudeBd(4) -
                            magnitudeBd(3)),
                    1.0e-6);
                        
            minco.setParameters(
                testPoints,
                optimizedTimes);
            
            Trajectory<5> metricTrajectory;
            
            minco.getTrajectory(
                metricTrajectory);
            
            if (metricTrajectory.getPieceNum() !=
                pieceN)
            {
                return false;
            }
        
            // Four scalar constraint-utilization features:
            //
            //   0: velocity
            //   1: body rate
            //   2: tilt
            //   3: thrust
            constexpr int featurePerNode =
                4;
        
            const int nodeCount =
                pieceN *
                (integralRes + 1);
        
            featureVector.resize(
                featurePerNode *
                nodeCount);
            
            int offset = 0;
            
            for (int pieceId = 0;
                 pieceId < pieceN;
                 ++pieceId)
            {
                const double duration =
                    optimizedTimes(pieceId);
            
                if (!std::isfinite(duration) ||
                    duration <= 0.0)
                {
                    featureVector.resize(0);
                    return false;
                }
            
                const double dt =
                    duration /
                    static_cast<double>(
                        integralRes);
                    
                const auto &piece =
                    metricTrajectory[pieceId];
                    
                for (int sampleId = 0;
                     sampleId <= integralRes;
                     ++sampleId)
                {
                    const double alpha =
                        static_cast<double>(sampleId) /
                        static_cast<double>(
                            integralRes);
                        
                    const double localTime =
                        alpha *
                        duration;
                        
                    const double trapezoidWeight =
                        (sampleId == 0 ||
                         sampleId == integralRes)
                            ? 0.5
                            : 1.0;
                        
                    // Makes J^T J approximate a time integral.
                    const double integrationWeight =
                        std::sqrt(
                            trapezoidWeight *
                            dt);
                        
                    const Eigen::Vector3d vel =
                        piece.getVel(
                            localTime);
                        
                    const Eigen::Vector3d acc =
                        piece.getAcc(
                            localTime);
                        
                    const Eigen::Vector3d jer =
                        piece.getJer(
                            localTime);
                        
                    double thrust = 0.0;
                        
                    Eigen::Vector4d quat =
                        Eigen::Vector4d::Zero();
                        
                    Eigen::Vector3d bodyRate =
                        Eigen::Vector3d::Zero();
                        
                    flatmap.forward(
                        vel,
                        acc,
                        jer,
                        0.0,
                        0.0,
                        thrust,
                        quat,
                        bodyRate);
                    
                    if (!vel.allFinite() ||
                        !acc.allFinite() ||
                        !jer.allFinite() ||
                        !std::isfinite(thrust) ||
                        !quat.allFinite() ||
                        !bodyRate.allFinite())
                    {
                        featureVector.resize(0);
                        return false;
                    }
                
                    const double velocityUtilization =
                        vel.squaredNorm() /
                        (velocityScale *
                         velocityScale);
                        
                    const double bodyRateUtilization =
                        bodyRate.squaredNorm() /
                        (bodyRateScale *
                         bodyRateScale);
                        
                    const double tiltUtilization =
                        (quat(1) * quat(1) +
                         quat(2) * quat(2)) /
                        (tiltScale *
                         tiltScale);
                        
                    const double thrustNormalized =
                        (thrust -
                         thrustMean) /
                        thrustRadius;
                        
                    const double thrustUtilization =
                        thrustNormalized *
                        thrustNormalized;
                        
                    if (!std::isfinite(
                            velocityUtilization) ||
                        !std::isfinite(
                            bodyRateUtilization) ||
                        !std::isfinite(
                            tiltUtilization) ||
                        !std::isfinite(
                            thrustUtilization))
                    {
                        featureVector.resize(0);
                        return false;
                    }
                
                    featureVector(offset + 0) =
                        integrationWeight *
                        velocityUtilization;
                
                    featureVector(offset + 1) =
                        integrationWeight *
                        bodyRateUtilization;
                
                    featureVector(offset + 2) =
                        integrationWeight *
                        tiltUtilization;
                
                    featureVector(offset + 3) =
                        integrationWeight *
                        thrustUtilization;
                
                    offset +=
                        featurePerNode;
                }
            }
        
            return featureVector.allFinite();
        }

        inline bool evaluateNominalDynamicsProximityWeights(
            Eigen::Matrix<double, 4, Eigen::Dynamic> &nodeWeights,
            Eigen::Vector4d &meanRatios,
            Eigen::Vector4d &maxRatios,
            Eigen::Vector4d &meanWeights,
            const double proximityPower)
        {
            nodeWeights.resize(4, 0);
        
            meanRatios.setZero();
            maxRatios.setZero();
            meanWeights.setZero();
        
            if (!optimizedStateValid ||
                pieceN <= 0 ||
                integralRes <= 0 ||
                optimizedTimes.size() != pieceN ||
                magnitudeBd.size() < 5 ||
                !std::isfinite(proximityPower) ||
                proximityPower <= 0.0)
            {
                return false;
            }
        
            const double velocityScale =
                std::max(
                    std::abs(magnitudeBd(0)),
                    1.0e-6);
                
            const double bodyRateScale =
                std::max(
                    std::abs(magnitudeBd(1)),
                    1.0e-6);
                
            const double thetaMax =
                std::max(
                    std::abs(magnitudeBd(2)),
                    1.0e-6);
                
            const double tiltScale =
                std::max(
                    std::sin(0.5 * thetaMax),
                    1.0e-6);
                
            const double thrustMean =
                0.5 *
                (magnitudeBd(3) +
                 magnitudeBd(4));
                
            const double thrustRadius =
                std::max(
                    0.5 *
                        std::abs(
                            magnitudeBd(4) -
                            magnitudeBd(3)),
                    1.0e-6);
                        
            minco.setParameters(
                optimizedPoints,
                optimizedTimes);
            
            Trajectory<5> nominalTrajectory;
            
            minco.getTrajectory(
                nominalTrajectory);
            
            if (nominalTrajectory.getPieceNum() !=
                pieceN)
            {
                return false;
            }
        
            const int nodeCount =
                pieceN *
                (integralRes + 1);
        
            nodeWeights.resize(
                4,
                nodeCount);
            
            int nodeId = 0;
            
            auto computeWeight =
                [&](const double ratio) -> double
            {
                const double clipped =
                    std::max(
                        0.0,
                        std::min(
                            ratio,
                            1.0));
                        
                return std::pow(
                    clipped,
                    proximityPower);
            };
        
            for (int pieceId = 0;
                 pieceId < pieceN;
                 ++pieceId)
            {
                const double duration =
                    optimizedTimes(pieceId);
            
                if (!std::isfinite(duration) ||
                    duration <= 0.0)
                {
                    return false;
                }
            
                const auto &piece =
                    nominalTrajectory[pieceId];
            
                for (int sampleId = 0;
                     sampleId <= integralRes;
                     ++sampleId)
                {
                    const double alpha =
                        static_cast<double>(sampleId) /
                        static_cast<double>(integralRes);
                
                    const double localTime =
                        alpha * duration;
                
                    const Eigen::Vector3d vel =
                        piece.getVel(localTime);
                
                    const Eigen::Vector3d acc =
                        piece.getAcc(localTime);
                
                    const Eigen::Vector3d jer =
                        piece.getJer(localTime);
                
                    double thrust = 0.0;
                
                    Eigen::Vector4d quat =
                        Eigen::Vector4d::Zero();
                
                    Eigen::Vector3d bodyRate =
                        Eigen::Vector3d::Zero();
                
                    flatmap.forward(
                        vel,
                        acc,
                        jer,
                        0.0,
                        0.0,
                        thrust,
                        quat,
                        bodyRate);
                    
                    if (!vel.allFinite() ||
                        !acc.allFinite() ||
                        !jer.allFinite() ||
                        !std::isfinite(thrust) ||
                        !quat.allFinite() ||
                        !bodyRate.allFinite())
                    {
                        return false;
                    }
                
                    Eigen::Vector4d ratios;
                
                    ratios(0) =
                        vel.norm() /
                        velocityScale;
                
                    ratios(1) =
                        bodyRate.norm() /
                        bodyRateScale;
                
                    ratios(2) =
                        std::sqrt(
                            quat(1) * quat(1) +
                            quat(2) * quat(2)) /
                        tiltScale;
                        
                    ratios(3) =
                        std::abs(
                            thrust -
                            thrustMean) /
                        thrustRadius;
                        
                    if (!ratios.allFinite())
                    {
                        return false;
                    }
                
                    Eigen::Vector4d weights;
                
                    for (int featureId = 0;
                         featureId < 4;
                         ++featureId)
                    {
                        weights(featureId) =
                            computeWeight(
                                ratios(featureId));
                    }
                
                    nodeWeights.col(nodeId) =
                        weights;
                
                    meanRatios +=
                        ratios;
                
                    meanWeights +=
                        weights;
                
                    maxRatios =
                        maxRatios.cwiseMax(
                            ratios);
                        
                    ++nodeId;
                }
            }
        
            if (nodeId != nodeCount)
            {
                return false;
            }
        
            meanRatios /=
                static_cast<double>(
                    nodeCount);
                
            meanWeights /=
                static_cast<double>(
                    nodeCount);
                
            return nodeWeights.allFinite() &&
                   meanRatios.allFinite() &&
                   maxRatios.allFinite() &&
                   meanWeights.allFinite();
        }

        inline bool computeSingleGaussNewtonDeformationMetric(
            const int pieceId,
            GaussNewtonDeformationMetric &metric,
            const double displacementStep,
            const double relativeDamping,
            const double proximityPower,
            const double maxCorridorAnisotropy,
            const Eigen::Matrix<double, 4, Eigen::Dynamic> &nodeWeights,
            const Eigen::Vector4d &meanRatios,
            const Eigen::Vector4d &maxRatios,
            const Eigen::Vector4d &meanWeights)
        {
            metric =
                GaussNewtonDeformationMetric();
        
            metric.displacementStepUsed =
                displacementStep;

            metric.proximityPowerUsed =
                proximityPower;

            metric.meanProximityRatios =
                meanRatios;

            metric.maxProximityRatios =
                maxRatios;

            metric.meanProximityWeights =
                meanWeights;
            metric.maxCorridorAnisotropyUsed =
                maxCorridorAnisotropy;

            auto fail =
                [&](const char *reason) -> bool
            {
                metric.valid = false;
                metric.failureReason = reason;
                return false;
            };
        
            if (!optimizedStateValid)
            {
                return fail(
                    "optimized_state_invalid");
            }
        
            if (pieceId < 0 ||
                pieceId >= pieceN)
            {
                return fail(
                    "invalid_piece_id");
            }
        
            if (!std::isfinite(displacementStep) ||
                displacementStep <= 0.0)
            {
                return fail(
                    "invalid_displacement_step");
            }

            if (!std::isfinite(maxCorridorAnisotropy) ||
                maxCorridorAnisotropy < 1.0)
            {
                return fail(
                    "invalid_max_corridor_anisotropy");
            }
        
            if (!std::isfinite(relativeDamping) ||
                relativeDamping < 0.0)
            {
                return fail(
                    "invalid_relative_damping");
            }
        
            if (!std::isfinite(proximityPower) ||
                proximityPower <= 0.0)
            {
                return fail(
                    "invalid_proximity_power");
            }

            Eigen::MatrixXd featureJacobian;
        
            bool jacobianAllocated =
                false;
        
            // ------------------------------------------------------------
            // Central finite difference of the DYNAMICS FEATURE VECTOR:
            //
            //     J(:,j) =
            //
            //       [phi(P + h D e_j)
            //        -
            //        phi(P - h D e_j)] / (2h)
            //
            // Only FIRST derivatives are needed.
            // ------------------------------------------------------------
            for (int axis = 0;
                 axis < 3;
                 ++axis)
            {
                Eigen::Matrix3Xd plusPoints;
                Eigen::Matrix3Xd minusPoints;
            
                if (!buildPerturbedPointsForPiece(
                        pieceId,
                        axis,
                        displacementStep,
                        plusPoints,
                        minusPoints))
                {
                    return fail(
                        "cartesian_perturbation_failed");
                }
            
                Eigen::VectorXd featurePlus;
                Eigen::VectorXd featureMinus;
            
                if (!evaluateConstraintUtilizationFeatureVector(
                        plusPoints,
                        featurePlus))
                {
                    return fail(
                        "feature_plus_failed");
                }
            
                if (!evaluateConstraintUtilizationFeatureVector(
                        minusPoints,
                        featureMinus))
                {
                    return fail(
                        "feature_minus_failed");
                }
            
                if (featurePlus.size() <= 0 ||
                    featurePlus.size() !=
                        featureMinus.size())
                {
                    return fail(
                        "feature_dimension_mismatch");
                }
            
                if (!jacobianAllocated)
                {
                    featureJacobian.resize(
                        featurePlus.size(),
                        3);
                    
                    jacobianAllocated =
                        true;
                }
            
                if (featurePlus.size() !=
                    featureJacobian.rows())
                {
                    return fail(
                        "feature_dimension_changed");
                }
            
                featureJacobian.col(axis) =
                    (featurePlus -
                     featureMinus) /
                    (2.0 *
                     displacementStep);
            }
        
            if (!jacobianAllocated ||
                !featureJacobian.allFinite())
            {
                return fail(
                    "feature_jacobian_invalid");
            }
        
            metric.jacobianFrobeniusNorm =
                featureJacobian.norm();

            constexpr int featurePerNode = 4;

            if (featureJacobian.rows() %
                    featurePerNode !=
                0)
            {
                return fail(
                    "gn_feature_row_count_invalid");
            }

            const int featureNodeCount =
                featureJacobian.rows() /
                featurePerNode;

            if (nodeWeights.rows() != 4 ||
                nodeWeights.cols() !=
                    featureNodeCount)
            {
                return fail(
                    "gn_proximity_weight_dimension_mismatch");
            }

            // ------------------------------------------------------------
            // Raw, unweighted GN Gram.
            // ------------------------------------------------------------
            Eigen::Matrix3d rawGram =
                featureJacobian.transpose() *
                featureJacobian;

            rawGram =
                (0.5 *
                 (rawGram +
                  rawGram.transpose()))
                    .eval();
                
            if (!rawGram.allFinite())
            {
                return fail(
                    "gn_raw_gram_nonfinite");
            }

            metric.rawGram =
                rawGram;

            // ------------------------------------------------------------
            // Construct W^(1/2) J.
            //
            // Each nominal proximity weight is frozen while taking the
            // Cartesian finite difference. Therefore
            //
            //     G_w = J^T W J
            //
            // remains positive semidefinite.
            // ------------------------------------------------------------
            Eigen::MatrixXd weightedJacobian =
                featureJacobian;

            for (int nodeId = 0;
                 nodeId < featureNodeCount;
                 ++nodeId)
            {
                const int row =
                    featurePerNode *
                    nodeId;
            
                const double sqrtVelocityWeight =
                    std::sqrt(
                        nodeWeights(0, nodeId));
                    
                const double sqrtBodyRateWeight =
                    std::sqrt(
                        nodeWeights(1, nodeId));
                    
                const double sqrtTiltWeight =
                    std::sqrt(
                        nodeWeights(2, nodeId));
                    
                const double sqrtThrustWeight =
                    std::sqrt(
                        nodeWeights(3, nodeId));
                    
                weightedJacobian.block<1, 3>(
                    row + 0,
                    0) *=
                    sqrtVelocityWeight;
                
                weightedJacobian.block<1, 3>(
                    row + 1,
                    0) *=
                    sqrtBodyRateWeight;
                
                weightedJacobian.block<1, 3>(
                    row + 2,
                    0) *=
                    sqrtTiltWeight;
                
                weightedJacobian.block<1, 3>(
                    row + 3,
                    0) *=
                    sqrtThrustWeight;
            }

            if (!weightedJacobian.allFinite())
            {
                return fail(
                    "gn_weighted_jacobian_nonfinite");
            }

            Eigen::Matrix3d gram =
                weightedJacobian.transpose() *
                weightedJacobian;

            gram =
                (0.5 *
                 (gram +
                  gram.transpose()))
                    .eval();
                
            if (!gram.allFinite())
            {
                return fail(
                    "gn_weighted_gram_nonfinite");
            }

            // ------------------------------------------------------------
            // Independent per-feature decomposition of J^T W J.
            // ------------------------------------------------------------
            Eigen::Matrix3d velocityGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d bodyRateGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d tiltGram =
                Eigen::Matrix3d::Zero();

            Eigen::Matrix3d thrustGram =
                Eigen::Matrix3d::Zero();

            for (int nodeId = 0;
                 nodeId < featureNodeCount;
                 ++nodeId)
            {
                const int row =
                    featurePerNode *
                    nodeId;
            
                const Eigen::Matrix<double, 1, 3>
                    velocityJacobian =
                        featureJacobian.block<1, 3>(
                            row + 0,
                            0);
                        
                const Eigen::Matrix<double, 1, 3>
                    bodyRateJacobian =
                        featureJacobian.block<1, 3>(
                            row + 1,
                            0);
                        
                const Eigen::Matrix<double, 1, 3>
                    tiltJacobian =
                        featureJacobian.block<1, 3>(
                            row + 2,
                            0);
                        
                const Eigen::Matrix<double, 1, 3>
                    thrustJacobian =
                        featureJacobian.block<1, 3>(
                            row + 3,
                            0);
                        
                velocityGram.noalias() +=
                    nodeWeights(0, nodeId) *
                    velocityJacobian.transpose() *
                    velocityJacobian;
                        
                bodyRateGram.noalias() +=
                    nodeWeights(1, nodeId) *
                    bodyRateJacobian.transpose() *
                    bodyRateJacobian;
                        
                tiltGram.noalias() +=
                    nodeWeights(2, nodeId) *
                    tiltJacobian.transpose() *
                    tiltJacobian;
                        
                thrustGram.noalias() +=
                    nodeWeights(3, nodeId) *
                    thrustJacobian.transpose() *
                    thrustJacobian;
            }

            velocityGram =
                (0.5 *
                 (velocityGram +
                  velocityGram.transpose()))
                    .eval();
                
            bodyRateGram =
                (0.5 *
                 (bodyRateGram +
                  bodyRateGram.transpose()))
                    .eval();
                
            tiltGram =
                (0.5 *
                 (tiltGram +
                  tiltGram.transpose()))
                    .eval();
                
            thrustGram =
                (0.5 *
                 (thrustGram +
                  thrustGram.transpose()))
                    .eval();
                
            if (!velocityGram.allFinite() ||
                !bodyRateGram.allFinite() ||
                !tiltGram.allFinite() ||
                !thrustGram.allFinite())
            {
                return fail(
                    "gn_weighted_component_nonfinite");
            }

            metric.velocityGram =
                velocityGram;

            metric.bodyRateGram =
                bodyRateGram;

            metric.tiltGram =
                tiltGram;

            metric.thrustGram =
                thrustGram;

            const Eigen::Matrix3d reconstructedGram =
                velocityGram +
                bodyRateGram +
                tiltGram +
                thrustGram;

            metric.decompositionRelativeError =
                (gram -
                 reconstructedGram)
                    .norm() /
                std::max(
                    gram.norm(),
                    1.0e-12);
                
            if (!std::isfinite(
                    metric.decompositionRelativeError))
            {
                return fail(
                    "gn_decomposition_error_nonfinite");
            }

            // ------------------------------------------------------------
            // Weighted contribution fractions.
            // ------------------------------------------------------------
            const double velocityTrace =
                velocityGram.trace();

            const double bodyRateTrace =
                bodyRateGram.trace();

            const double tiltTrace =
                tiltGram.trace();

            const double thrustTrace =
                thrustGram.trace();

            const double totalFeatureTrace =
                velocityTrace +
                bodyRateTrace +
                tiltTrace +
                thrustTrace;

            if (!std::isfinite(totalFeatureTrace) ||
                totalFeatureTrace < 0.0)
            {
                return fail(
                    "gn_component_trace_invalid");
            }

            if (totalFeatureTrace > 1.0e-12)
            {
                metric.velocityTraceFraction =
                    velocityTrace /
                    totalFeatureTrace;
            
                metric.bodyRateTraceFraction =
                    bodyRateTrace /
                    totalFeatureTrace;
            
                metric.tiltTraceFraction =
                    tiltTrace /
                    totalFeatureTrace;
            
                metric.thrustTraceFraction =
                    thrustTrace /
                    totalFeatureTrace;
            }

            auto computeDirectionality =
                [](const Eigen::Matrix3d &component)
                    -> double
            {
                const double trace =
                    component.trace();
            
                if (!std::isfinite(trace) ||
                    trace <= 1.0e-12)
                {
                    return 0.0;
                }
            
                const Eigen::Matrix3d isotropicPart =
                    (trace / 3.0) *
                    Eigen::Matrix3d::Identity();
            
                return
                    (component -
                     isotropicPart)
                        .norm() /
                    trace;
            };

            metric.velocityDirectionality =
                computeDirectionality(
                    velocityGram);
                
            metric.bodyRateDirectionality =
                computeDirectionality(
                    bodyRateGram);
                
            metric.tiltDirectionality =
                computeDirectionality(
                    tiltGram);
                
            metric.thrustDirectionality =
                computeDirectionality(
                    thrustGram);
                
            metric.gram =
                gram;
        
            // ------------------------------------------------------------
            // The isotropic prior uses the RAW sensitivity scale.
            //
            // This is intentional:
            //
            // If every nominal constraint is far from its limit, W becomes
            // small, while the isotropic prior remains.  The final metric
            // then naturally approaches I.
            // ------------------------------------------------------------
            const double rawGramScale =
                rawGram.trace() /
                3.0;

            if (!std::isfinite(rawGramScale) ||
                rawGramScale < 0.0)
            {
                return fail(
                    "gn_raw_scale_invalid");
            }

            if (rawGramScale <= 1.0e-12)
            {
                metric.stiffness.setIdentity();
                metric.utility.setIdentity();
            
                metric.stiffnessEigenvalues.setOnes();
                metric.utilityEigenvalues.setOnes();
            
                metric.principalDirection =
                    Eigen::Vector3d::UnitX();
            
                metric.dampingUsed =
                    1.0;
            
                metric.rawAnisotropy =
                    1.0;
            
                metric.anisotropy =
                    1.0;
            
                metric.principalGap =
                    1.0;
            
                metric.valid =
                    true;
            
                metric.failureReason =
                    "none";
            
                return true;
            }

            const double damping =
                std::max(
                    relativeDamping *
                        rawGramScale,
                    1.0e-12);
                
            metric.dampingUsed =
                damping;
                
            // Raw GN anisotropy, for A/B comparison.
            const Eigen::Matrix3d rawStiffness =
                rawGram +
                damping *
                    Eigen::Matrix3d::Identity();
                
            Eigen::SelfAdjointEigenSolver<
                Eigen::Matrix3d>
                rawSolver(
                    rawStiffness);
                
            if (rawSolver.info() !=
                Eigen::Success)
            {
                return fail(
                    "gn_raw_eigensolver_failed");
            }

            const Eigen::Vector3d rawEigenvalues =
                rawSolver.eigenvalues();

            if (!rawEigenvalues.allFinite() ||
                rawEigenvalues.minCoeff() <= 0.0)
            {
                return fail(
                    "gn_raw_eigenvalues_invalid");
            }

            metric.rawAnisotropy =
                rawEigenvalues(2) /
                rawEigenvalues(0);

            // This is the actual proximity-weighted stiffness.
            const Eigen::Matrix3d stiffness =
                gram +
                damping *
                    Eigen::Matrix3d::Identity();
                
            if (!stiffness.allFinite())
            {
                return fail(
                    "gn_stiffness_nonfinite");
            }
        
            Eigen::SelfAdjointEigenSolver<
                Eigen::Matrix3d>
                stiffnessSolver(
                    stiffness);
                
            if (stiffnessSolver.info() !=
                Eigen::Success)
            {
                return fail(
                    "gn_stiffness_eigensolver_failed");
            }
        
            const Eigen::Vector3d stiffnessEigenvalues =
                stiffnessSolver.eigenvalues();
        
            if (!stiffnessEigenvalues.allFinite() ||
                stiffnessEigenvalues.minCoeff() <= 0.0)
            {
                return fail(
                    "gn_stiffness_not_positive");
            }
        
            metric.stiffness =
                stiffness;
        
            metric.stiffnessEigenvalues =
                stiffnessEigenvalues;
        
            // ------------------------------------------------------------
            // determinant-normalized inverse stiffness:
            //
            //     S ~ K^{-1}
            //
            // with
            //
            //     det(S) = 1.
            //
            // Use log-space geometric mean to avoid scale problems.
            // ------------------------------------------------------------
            const double meanLogStiffness =
                stiffnessEigenvalues.array()
                    .log()
                    .mean();
        
            const double geometricMeanStiffness =
                std::exp(
                    meanLogStiffness);
                
            Eigen::Vector3d utilityEigenvalues;
                
            for (int i = 0;
                 i < 3;
                 ++i)
            {
                utilityEigenvalues(i) =
                    geometricMeanStiffness /
                    stiffnessEigenvalues(i);
            }
        
            if (!utilityEigenvalues.allFinite() ||
                utilityEigenvalues.minCoeff() <= 0.0)
            {
                return fail(
                    "gn_utility_eigenvalues_invalid");
            }
        
            Eigen::Matrix3d utility =
                stiffnessSolver.eigenvectors() *
                utilityEigenvalues.asDiagonal() *
                stiffnessSolver.eigenvectors().transpose();
        
            utility =
                (0.5 *
                 (utility +
                  utility.transpose()))
                    .eval();
                
            if (!utility.allFinite())
            {
                return fail(
                    "gn_utility_nonfinite");
            }
        
            metric.utility =
                utility;
        
            // Eigenvalues of stiffness are ascending:
            //
            // lambda0 <= lambda1 <= lambda2.
            //
            // Therefore utility eigenvalues above are descending in the
            // SAME eigenvector basis:
            //
            // mu0 >= mu1 >= mu2.
            //
            // The easiest deformation direction is eigenvector column 0
            // of the STIFFNESS eigensystem.
            metric.utilityEigenvalues =
                utilityEigenvalues;
        
            // ------------------------------------------------------------
            // Order-preserving log-spectrum compression.
            //
            // Natural utility eigenvalues are ordered:
            //
            //     mu_0 >= mu_1 >= mu_2 > 0.
            //
            // We retain:
            //   * eigenvectors,
            //   * eigenvalue ordering,
            //   * determinant = 1,
            //
            // while bounding:
            //
            //     mu_max / mu_min <= maxCorridorAnisotropy.
            // ------------------------------------------------------------
            const double naturalUtilityAnisotropy =
                utilityEigenvalues(0) /
                utilityEigenvalues(2);

            if (!std::isfinite(
                    naturalUtilityAnisotropy) ||
                naturalUtilityAnisotropy < 1.0)
            {
                return fail(
                    "natural_utility_anisotropy_invalid");
            }

            double compressionAlpha =
                1.0;

            if (naturalUtilityAnisotropy >
                    maxCorridorAnisotropy &&
                naturalUtilityAnisotropy >
                    1.0 + 1.0e-12)
            {
                compressionAlpha =
                    std::log(
                        maxCorridorAnisotropy) /
                    std::log(
                        naturalUtilityAnisotropy);
            }

            compressionAlpha =
                std::max(
                    0.0,
                    std::min(
                        compressionAlpha,
                        1.0));
                    
            metric.spectrumCompressionAlpha =
                compressionAlpha;
                    
            // Work in log-space.
            // Re-centering guarantees det(S_c) = 1 even in the presence
            // of small floating-point drift.
            Eigen::Vector3d logUtility;
                    
            for (int i = 0;
                 i < 3;
                 ++i)
            {
                logUtility(i) =
                    std::log(
                        utilityEigenvalues(i));
            }

            const double meanLogUtility =
                logUtility.mean();

            Eigen::Vector3d compressedLogUtility;

            for (int i = 0;
                 i < 3;
                 ++i)
            {
                compressedLogUtility(i) =
                    compressionAlpha *
                    (logUtility(i) -
                     meanLogUtility);
            }

            Eigen::Vector3d corridorUtilityEigenvalues;

            for (int i = 0;
                 i < 3;
                 ++i)
            {
                corridorUtilityEigenvalues(i) =
                    std::exp(
                        compressedLogUtility(i));
            }

            if (!corridorUtilityEigenvalues.allFinite() ||
                corridorUtilityEigenvalues.minCoeff() <=
                    0.0)
            {
                return fail(
                    "corridor_utility_eigenvalues_invalid");
            }

            Eigen::Matrix3d corridorUtility =
                stiffnessSolver.eigenvectors() *
                corridorUtilityEigenvalues.asDiagonal() *
                stiffnessSolver.eigenvectors().transpose();

            corridorUtility =
                (0.5 *
                 (corridorUtility +
                  corridorUtility.transpose()))
                    .eval();
                
            if (!corridorUtility.allFinite())
            {
                return fail(
                    "corridor_utility_nonfinite");
            }

            metric.corridorUtility =
                corridorUtility;

            metric.corridorUtilityEigenvalues =
                corridorUtilityEigenvalues;

            metric.corridorAnisotropy =
                corridorUtilityEigenvalues(0) /
                corridorUtilityEigenvalues(2);

            metric.principalDirection =
                stiffnessSolver
                    .eigenvectors()
                    .col(0);
        
            metric.anisotropy =
                stiffnessEigenvalues(2) /
                stiffnessEigenvalues(0);
        
            metric.principalGap =
                stiffnessEigenvalues(1) /
                stiffnessEigenvalues(0);
        
            // Stable visualization sign.
            Eigen::Vector3d leftPoint;
            Eigen::Vector3d rightPoint;
        
            if (pieceId == 0)
            {
                leftPoint =
                    headPVA.col(0);
            }
            else
            {
                leftPoint =
                    optimizedPoints.col(
                        pieceId - 1);
            }
        
            if (pieceId ==
                pieceN - 1)
            {
                rightPoint =
                    tailPVA.col(0);
            }
            else
            {
                rightPoint =
                    optimizedPoints.col(
                        pieceId);
            }
        
            const Eigen::Vector3d tangent =
                rightPoint -
                leftPoint;
        
            if (tangent.norm() > 1.0e-9 &&
                metric.principalDirection.dot(
                    tangent) < 0.0)
            {
                metric.principalDirection *=
                    -1.0;
            }
        
            metric.valid =
                true;
        
            metric.failureReason =
                "none";
        
            return true;
        }

        inline bool evaluatePieceDeformationCost(
            const int pieceId,
            const Eigen::Vector3d &deformation,
            const DeformationMetricObjective objectiveMode,
            double &cost)
        {
            Eigen::Matrix3Xd testPoints;
        
            if (!buildDeformedPointsForPiece(
                    pieceId,
                    deformation,
                    testPoints))
            {
                cost = INFINITY;
                return false;
            }
        
            Eigen::Matrix3Xd dummyGradient;
        
            cost =
                evaluateObjectiveAtPoints(
                    testPoints,
                    dummyGradient,
                    objectiveMode);
                
            return std::isfinite(cost);
        }

        inline bool computeScalarCostStiffness(
            const int pieceId,
            const double displacementStep,
            const DeformationMetricObjective objectiveMode,
            Eigen::Matrix3d &stiffness)
        {
            stiffness.setZero();
        
            if (!optimizedStateValid ||
                displacementStep <= 0.0 ||
                !std::isfinite(displacementStep))
            {
                return false;
            }
        
            const double h =
                displacementStep;
        
            const double h2 =
                h * h;
        
            double cost0 = INFINITY;
        
            if (!evaluatePieceDeformationCost(
                    pieceId,
                    Eigen::Vector3d::Zero(),
                    objectiveMode,
                    cost0))
            {
                return false;
            }
        
            // ------------------------------------------------------------
            // Diagonal entries
            //
            // K_ii =
            //
            //   [F(+h e_i) - 2 F(0) + F(-h e_i)] / h^2
            // ------------------------------------------------------------
            for (int axis = 0;
                 axis < 3;
                 ++axis)
            {
                Eigen::Vector3d delta =
                    Eigen::Vector3d::Zero();
            
                delta(axis) =
                    h;
            
                double costPlus =
                    INFINITY;
            
                double costMinus =
                    INFINITY;
            
                if (!evaluatePieceDeformationCost(
                        pieceId,
                        delta,
                        objectiveMode,
                        costPlus))
                {
                    return false;
                }
            
                if (!evaluatePieceDeformationCost(
                        pieceId,
                        -delta,
                        objectiveMode,
                        costMinus))
                {
                    return false;
                }
            
                stiffness(axis, axis) =
                    (costPlus -
                     2.0 * cost0 +
                     costMinus) /
                    h2;
            }
        
            // ------------------------------------------------------------
            // Off-diagonal entries
            //
            // K_ij =
            //
            // [ F(+i,+j)
            // - F(+i,-j)
            // - F(-i,+j)
            // + F(-i,-j) ] / (4 h^2)
            // ------------------------------------------------------------
            for (int i = 0;
                 i < 3;
                 ++i)
            {
                for (int j = i + 1;
                     j < 3;
                     ++j)
                {
                    Eigen::Vector3d deltaPP =
                        Eigen::Vector3d::Zero();
                
                    Eigen::Vector3d deltaPM =
                        Eigen::Vector3d::Zero();
                
                    Eigen::Vector3d deltaMP =
                        Eigen::Vector3d::Zero();
                
                    Eigen::Vector3d deltaMM =
                        Eigen::Vector3d::Zero();
                
                    deltaPP(i) = h;
                    deltaPP(j) = h;
                
                    deltaPM(i) = h;
                    deltaPM(j) = -h;
                
                    deltaMP(i) = -h;
                    deltaMP(j) = h;
                
                    deltaMM(i) = -h;
                    deltaMM(j) = -h;
                
                    double costPP =
                        INFINITY;
                
                    double costPM =
                        INFINITY;
                
                    double costMP =
                        INFINITY;
                
                    double costMM =
                        INFINITY;
                
                    if (!evaluatePieceDeformationCost(
                            pieceId,
                            deltaPP,
                            objectiveMode,
                            costPP) ||
                        !evaluatePieceDeformationCost(
                            pieceId,
                            deltaPM,
                            objectiveMode,
                            costPM) ||
                        !evaluatePieceDeformationCost(
                            pieceId,
                            deltaMP,
                            objectiveMode,
                            costMP) ||
                        !evaluatePieceDeformationCost(
                            pieceId,
                            deltaMM,
                            objectiveMode,
                            costMM))
                    {
                        return false;
                    }
                
                    const double value =
                        (costPP -
                         costPM -
                         costMP +
                         costMM) /
                        (4.0 * h2);
                        
                    stiffness(i, j) =
                        value;
                        
                    stiffness(j, i) =
                        value;
                }
            }
        
            return stiffness.allFinite();
        }

        inline int getXiBlockOffset(const int pointId) const
        {
            int offset = 0;

            for (int i = 0; i < pointId; ++i)
            {
                const int polyId = vPolyIdx(i);
                offset += vPolytopes[polyId].cols();
            }

            return offset;
        }

        inline bool computePointJacobianWrtXi(
            const int pointId,
            const Eigen::VectorXd &fullX,
            Eigen::MatrixXd &jacobian) const
        {
            if (pointId < 0 ||
                pointId >= static_cast<int>(vPolyIdx.size()))
            {
                return false;
            }

            const int polyId = vPolyIdx(pointId);
            const int k = vPolytopes[polyId].cols();

            if (k < 2)
            {
                return false;
            }

            const int offset = getXiBlockOffset(pointId);
            const int begin = temporalDim + offset;

            if (begin + k > fullX.size())
            {
                return false;
            }

            const Eigen::VectorXd xi =
                fullX.segment(begin, k);

            const double xiNorm = xi.norm();

            if (!std::isfinite(xiNorm) ||
                xiNorm <= 1.0e-10)
            {
                return false;
            }

            // u = xi / ||xi||
            const Eigen::VectorXd u = xi / xiNorm;

            // GCOPTER's forwardP() uses only the first k - 1 entries.
            const Eigen::VectorXd r = u.head(k - 1);

            // p = v0 + V * (r .* r)
            const auto V =
                vPolytopes[polyId].rightCols(k - 1);

            // dp / du
            Eigen::MatrixXd dPdu =
                Eigen::MatrixXd::Zero(3, k);

            dPdu.leftCols(k - 1) =
                2.0 * (V * r.asDiagonal());

            // du / dxi =
            // (I - u*u^T) / ||xi||
            const Eigen::MatrixXd projector =
                Eigen::MatrixXd::Identity(k, k) -
                u * u.transpose();

            jacobian =
                dPdu * projector / xiNorm;

            return jacobian.allFinite();
        }

        inline bool buildPieceDeformationMap(
            const int pieceId,
            const Eigen::VectorXd &fullX,
            Eigen::MatrixXd &deformationMap,
            const double jacobianDamping) const
        {
            if (pieceId < 0 || pieceId >= pieceN)
            {
                return false;
            }

            const int totalDim =
                temporalDim + spatialDim;

            deformationMap =
                Eigen::MatrixXd::Zero(totalDim, 3);

            int addedPointCount = 0;

            auto addPoint =
                [&](const int pointId) -> bool
            {
                Eigen::MatrixXd J;

                if (!computePointJacobianWrtXi(
                        pointId, fullX, J))
                {
                    return false;
                }

                Eigen::Matrix3d gram =
                    J * J.transpose();

                const double gramScale =
                    std::max(gram.trace() / 3.0,
                             1.0e-12);

                gram.diagonal().array() +=
                    jacobianDamping * gramScale;

                Eigen::LDLT<Eigen::Matrix3d> ldlt(gram);

                if (ldlt.info() != Eigen::Success)
                {
                    return false;
                }

                const Eigen::MatrixXd pseudoInverse =
                    J.transpose() *
                    ldlt.solve(
                        Eigen::Matrix3d::Identity());

                if (!pseudoInverse.allFinite())
                {
                    return false;
                }

                const int polyId =
                    vPolyIdx(pointId);

                const int k =
                    vPolytopes[polyId].cols();

                const int offset =
                    getXiBlockOffset(pointId);

                deformationMap.block(
                    temporalDim + offset,
                    0,
                    k,
                    3) = pseudoInverse;

                ++addedPointCount;

                return true;
            };

            // Piece i lies between:
            //
            // start / point_(i-1)
            // and
            // point_i / goal.
            //
            // We move both movable ends in the same Cartesian direction.
            if (pieceId > 0)
            {
                addPoint(pieceId - 1);
            }

            if (pieceId < pieceN - 1)
            {
                addPoint(pieceId);
            }

            return addedPointCount > 0;
        }

        inline bool computeSingleDeformationMetric(
            const int pieceId,
            DeformationMetric &metric,
            const double displacementStep,
            const double jacobianDamping,
            const double maxAnisotropy,
            const DeformationMetricObjective objectiveMode)
        {
            metric = DeformationMetric();
           
            (void)jacobianDamping;

            auto fail =
                [&](const char *reason) -> bool
            {
                metric.valid = false;
                metric.failureReason = reason;
                return false;
            };
        
            // ------------------------------------------------------------
            // Stage 1: validate the saved nominal solution.
            // ------------------------------------------------------------
            if (!optimizedStateValid)
            {
                return fail("optimized_state_invalid");
            }
        
            if (displacementStep <= 0.0 ||
                !std::isfinite(displacementStep))
            {
                return fail("invalid_displacement_step");
            }
        
            // ------------------------------------------------------------
            // Stage 2: construct
            //
            //     delta x = D * delta p
            //
            // for this trajectory piece.
            // ------------------------------------------------------------
            Eigen::MatrixXd D;
        
            // ------------------------------------------------------------
            // Stage 2:
            // Construct the projected Hessian directly in Cartesian
            // MINCO waypoint space.
            //
            //     K_i = D_i^T H_P D_i
            //
            // This avoids GCOPTER's nonlinear xi parameterization.
            // ------------------------------------------------------------
            Eigen::Matrix3d Kraw =
                Eigen::Matrix3d::Zero();

            for (int axis = 0;
                 axis < 3;
                 ++axis)
            {
                Eigen::Matrix3Xd plusPoints;
                Eigen::Matrix3Xd minusPoints;

                if (!buildPerturbedPointsForPiece(
                        pieceId,
                        axis,
                        displacementStep,
                        plusPoints,
                        minusPoints))
                {
                    return fail(
                        "cartesian_perturbation_failed");
                }

                Eigen::Matrix3Xd gradientPlus;
                Eigen::Matrix3Xd gradientMinus;

                const double costPlus =
                    evaluateObjectiveAtPoints(
                        plusPoints,
                        gradientPlus,
                        objectiveMode);
                    
                const double costMinus =
                    evaluateObjectiveAtPoints(
                        minusPoints,
                        gradientMinus,
                        objectiveMode);
                    
                if (!std::isfinite(costPlus) ||
                    !std::isfinite(costMinus))
                {
                    return fail(
                        "cartesian_perturbed_cost_nonfinite");
                }

                if (!gradientPlus.allFinite() ||
                    !gradientMinus.allFinite())
                {
                    return fail(
                        "cartesian_gradient_nonfinite");
                }

                const Eigen::Vector3d projectedGradientPlus =
                    projectPointGradientToPiece(
                        pieceId,
                        gradientPlus);

                const Eigen::Vector3d projectedGradientMinus =
                    projectPointGradientToPiece(
                        pieceId,
                        gradientMinus);

                Kraw.col(axis) =
                    (projectedGradientPlus -
                     projectedGradientMinus) /
                    (2.0 * displacementStep);
            }

            if (!Kraw.allFinite())
            {
                return fail(
                    "cartesian_stiffness_nonfinite");
            }

            // ------------------------------------------------------------
            // A true Hessian must be symmetric.
            //
            // Measure the discrepancy BEFORE symmetrization. This is a
            // useful numerical diagnostic of finite-difference quality.
            // ------------------------------------------------------------
            const double symmetryError =
                (Kraw - Kraw.transpose()).norm() /
                std::max(
                    Kraw.norm(),
                    1.0e-12);

            const Eigen::Matrix3d K =
                (0.5 *
                 (Kraw + Kraw.transpose()))
                    .eval();
            
            Eigen::Matrix3d scalarK;

            if (!computeScalarCostStiffness(
                    pieceId,
                    displacementStep,
                    objectiveMode,
                    scalarK))
            {
                return fail(
                    "scalar_cost_stiffness_failed");
            }

            metric.scalarStiffness =
                scalarK;

            // Compare the symmetrized gradient-difference Hessian with
            // the scalar-cost finite-difference Hessian.
            metric.gradientScalarRelativeError =
                (K - scalarK).norm() /
                std::max(
                    scalarK.norm(),
                    1.0e-12);

            if (!K.allFinite())
            {
                return fail(
                    "symmetric_stiffness_nonfinite");
            }

            metric.stiffness =
                scalarK;
        
            metric.symmetryError =
                symmetryError;
            // // ------------------------------------------------------------
            // // Stage 3: projected Hessian
            // //
            // //     K = D^T H D
            // //
            // // We never explicitly form the large Hessian H.
            // // ------------------------------------------------------------
            // Eigen::Matrix3d K =
            //     Eigen::Matrix3d::Zero();
        
            // for (int axis = 0; axis < 3; ++axis)
            // {
            //     const Eigen::VectorXd perturbation =
            //         displacementStep * D.col(axis);
            
            //     if (!perturbation.allFinite())
            //     {
            //         return fail("perturbation_nonfinite");
            //     }
            
            //     Eigen::VectorXd xPlus =
            //         optimizedX + perturbation;
            
            //     Eigen::VectorXd xMinus =
            //         optimizedX - perturbation;
            
            //     Eigen::VectorXd gPlus;
            //     Eigen::VectorXd gMinus;
            
            //     const double costPlus =
            //         evaluateObjectiveAt(xPlus, gPlus);
            
            //     const double costMinus =
            //         evaluateObjectiveAt(xMinus, gMinus);
            
            //     if (!std::isfinite(costPlus) ||
            //         !std::isfinite(costMinus))
            //     {
            //         return fail("perturbed_cost_nonfinite");
            //     }
            
            //     if (gPlus.size() != optimizedX.size() ||
            //         gMinus.size() != optimizedX.size())
            //     {
            //         return fail("perturbed_gradient_bad_dimension");
            //     }
            
            //     if (!gPlus.allFinite() ||
            //         !gMinus.allFinite())
            //     {
            //         return fail("perturbed_gradient_nonfinite");
            //     }
            
            //     const Eigen::VectorXd hessianVectorProduct =
            //         (gPlus - gMinus) /
            //         (2.0 * displacementStep);
            
            //     K.col(axis) =
            //         D.transpose() *
            //         hessianVectorProduct;
            // }
        
            // // Central finite differences may introduce a small
            // // antisymmetric numerical component.
            // const Eigen::Matrix3d rawK = K;

            // const double symmetryError =
            //     (rawK - rawK.transpose()).norm() /
            //     std::max(rawK.norm(), 1.0e-12);

            // const Eigen::Matrix3d symmetricK =
            //     (0.5 * (rawK + rawK.transpose())).eval();

            // K = symmetricK;
        
            // ROS_INFO_STREAM("piece " << pieceId
            //     << " raw K symmetry error = "
            //     << symmetryError);

            // if (!K.allFinite())
            // {
            //     return fail("stiffness_nonfinite");
            // }
        
            // metric.stiffness = K;
        
            // ------------------------------------------------------------
            // Stage 4: eigendecomposition of the Cartesian stiffness.
            // ------------------------------------------------------------
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
                stiffnessSolver(scalarK);
        
            if (stiffnessSolver.info() != Eigen::Success)
            {
                return fail("stiffness_eigensolver_failed");
            }
        
            const Eigen::Vector3d rawEigenvalues =
                stiffnessSolver.eigenvalues();
        
            if (!rawEigenvalues.allFinite())
            {
                return fail("stiffness_eigenvalues_nonfinite");
            }
        
            metric.rawStiffnessEigenvalues =
                rawEigenvalues;
            
            const double rawPositiveFloor =
                std::max(
                    rawEigenvalues.cwiseAbs().maxCoeff() * 1.0e-9,
                    1.0e-12);
                
            Eigen::Vector3d rawPositiveEigenvalues;
                
            for (int i = 0; i < 3; ++i)
            {
                rawPositiveEigenvalues(i) =
                    std::max(
                        rawEigenvalues(i),
                        rawPositiveFloor);
            }

            metric.rawAnisotropy =
                rawPositiveEigenvalues.maxCoeff() /
                rawPositiveEigenvalues.minCoeff();
            // ------------------------------------------------------------
            // Stage 5: positive regularization.
            //
            // The complete GCOPTER objective is nonconvex and uses smoothed
            // penalties, so the numerical local Hessian can contain negative
            // eigenvalues.  We project it to an SPD directional stiffness
            // before defining the inverse metric.
            // ------------------------------------------------------------
            const double spectrumScale =
                std::max(
                    rawEigenvalues.cwiseAbs().maxCoeff(),
                    1.0e-9);
                
            const double allowedAnisotropy =
                std::max(maxAnisotropy, 1.0);
                
            const double eigenFloor =
                spectrumScale /
                allowedAnisotropy;
                
            Eigen::Vector3d regularizedEigenvalues;
                
            for (int i = 0; i < 3; ++i)
            {
                regularizedEigenvalues(i) =
                    std::max(
                        rawEigenvalues(i),
                        eigenFloor);
            }
        
            if (!regularizedEigenvalues.allFinite() ||
                regularizedEigenvalues.minCoeff() <= 0.0)
            {
                return fail("regularized_stiffness_invalid");
            }
        
            metric.regularizedStiffnessEigenvalues =
                regularizedEigenvalues;
        
            // ------------------------------------------------------------
            // Stage 6: construct the trajectory-utility metric.
            //
            // Raw:
            //
            //     S_raw = K_reg^{-1}.
            //
            // We want det(S) = 1 so that S expresses directional preference
            // only, not an arbitrary cost scale.
            //
            // DO NOT compute det(S_raw) directly:
            //
            //     det(S_raw) = 1 / (lambda1 lambda2 lambda3)
            //
            // can easily be extremely small or large even for a perfectly
            // valid SPD matrix.
            //
            // Instead, if
            //
            //     g = (lambda1 lambda2 lambda3)^(1/3),
            //
            // then the normalized utility eigenvalues are exactly
            //
            //     mu_i = g / lambda_i.
            //
            // We compute g in log-space for numerical robustness.
            // ------------------------------------------------------------
            double meanLogStiffness = 0.0;
        
            for (int i = 0; i < 3; ++i)
            {
                const double lambda =
                    regularizedEigenvalues(i);
            
                if (!std::isfinite(lambda) ||
                    lambda <= 0.0)
                {
                    return fail("stiffness_log_invalid");
                }
            
                meanLogStiffness +=
                    std::log(lambda);
            }
        
            meanLogStiffness /= 3.0;
        
            const double geometricMeanStiffness =
                std::exp(meanLogStiffness);
        
            if (!std::isfinite(geometricMeanStiffness) ||
                geometricMeanStiffness <= 0.0)
            {
                return fail("stiffness_geometric_mean_invalid");
            }
        
            Eigen::Vector3d utilityEigenvalues;
        
            for (int i = 0; i < 3; ++i)
            {
                utilityEigenvalues(i) =
                    geometricMeanStiffness /
                    regularizedEigenvalues(i);
            }
        
            if (!utilityEigenvalues.allFinite() ||
                utilityEigenvalues.minCoeff() <= 0.0)
            {
                return fail("utility_eigenvalues_invalid");
            }
        
            Eigen::Matrix3d utility =
                stiffnessSolver.eigenvectors() *
                utilityEigenvalues.asDiagonal() *
                stiffnessSolver.eigenvectors().transpose();
        
            utility =
                0.5 *
                (utility + utility.transpose());
        
            if (!utility.allFinite())
            {
                return fail("utility_nonfinite");
            }
        
            // ------------------------------------------------------------
            // Stage 7: recover the principal easy-deformation direction.
            // ------------------------------------------------------------
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
                utilitySolver(utility);
        
            if (utilitySolver.info() != Eigen::Success)
            {
                return fail("utility_eigensolver_failed");
            }
        
            const Eigen::Vector3d finalUtilityEigenvalues =
                utilitySolver.eigenvalues();
        
            if (!finalUtilityEigenvalues.allFinite() ||
                finalUtilityEigenvalues.minCoeff() <= 0.0)
            {
                return fail("final_utility_eigenvalues_invalid");
            }
        
            metric.utility =
                utility;
        
            metric.principalDirection =
                utilitySolver.eigenvectors().col(2);
        
            metric.anisotropy =
                finalUtilityEigenvalues(2) /
                std::max(
                    finalUtilityEigenvalues(0),
                    1.0e-12);
                
            // ------------------------------------------------------------
            // Stage 8: choose a stable sign for visualization/logging.
            //
            // S itself is invariant to u -> -u, but a stable sign makes the
            // printed principal direction easier to compare across runs.
            // ------------------------------------------------------------
            Eigen::Vector3d leftPoint;
            Eigen::Vector3d rightPoint;
                
            if (pieceId == 0)
            {
                leftPoint =
                    headPVA.col(0);
            }
            else
            {
                leftPoint =
                    optimizedPoints.col(pieceId - 1);
            }
        
            if (pieceId == pieceN - 1)
            {
                rightPoint =
                    tailPVA.col(0);
            }
            else
            {
                rightPoint =
                    optimizedPoints.col(pieceId);
            }
        
            const Eigen::Vector3d tangent =
                rightPoint - leftPoint;
        
            if (tangent.norm() > 1.0e-9 &&
                metric.principalDirection.dot(tangent) < 0.0)
            {
                metric.principalDirection *= -1.0;
            }
        
            metric.valid = true;
            metric.failureReason = "none";
        
            return true;
        }

        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER_PolytopeSFC::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(3, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

    public:
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {
            rho = timeWeight;
            headPVA = initialPVA;
            tailPVA = terminalPVA;

            // A new planning problem invalidates the previous optimization
            // snapshot.
            optimizedStateValid = false;
            optimizedCost = INFINITY;
            optimizedX.resize(0);
            optimizedPoints.resize(3, 0);
            optimizedTimes.resize(0);

            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd =
                magnitudeBounds;

            basePenaltyWt =
                penaltyWeights;

            penaltyWt =
                basePenaltyWt;

            physicalPm =
                physicalParams;
            allocSpeed = magnitudeBd(0) * 3.0;

            // getShortestPath(headPVA.col(0), tailPVA.col(0),
            //                 vPolytopes, smoothEps, shortPath);
            // const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
            // pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            // pieceIdx.array() += 1;
            // pieceN = pieceIdx.sum();

            getShortestPath(headPVA.col(0), tailPVA.col(0),
                            vPolytopes, smoothEps, shortPath);
                    
            // ------------------------------------------------------------
            // Robust setup boundary.
            //
            // getShortestPath() currently has no failure return value.
            // A numerically failed shortest-path solve may therefore leave
            // non-finite coordinates in `shortPath`.
            //
            // Never allow such values to reach the floating-point -> int
            // conversion used to build pieceIdx.  In particular, production
            // uses lengthPerPiece = infinity, so NaN / infinity can otherwise
            // become an invalid negative Eigen segment length in setInitial().
            // ------------------------------------------------------------
            if (shortPath.cols() != polyN + 1 ||
                !shortPath.allFinite())
            {
                return false;
            }
            
            if (!(lengthPerPiece > 0.0) ||
                std::isnan(lengthPerPiece))
            {
                return false;
            }
            
            const Eigen::Matrix3Xd deltas =
                shortPath.rightCols(polyN) -
                shortPath.leftCols(polyN);
            
            if (!deltas.allFinite())
            {
                return false;
            }
            
            // Production commonly uses +infinity to request exactly one
            // trajectory piece per corridor.  Handle that intention directly
            // instead of relying on finite_length / infinity -> 0.
            if (std::isinf(lengthPerPiece))
            {
                pieceIdx =
                    Eigen::VectorXi::Ones(polyN);
            }
            else
            {
                const Eigen::RowVectorXd pieceRatios =
                    deltas.colwise().norm() /
                    lengthPerPiece;
            
                if (!pieceRatios.allFinite())
                {
                    return false;
                }
            
                pieceIdx =
                    pieceRatios
                        .cast<int>()
                        .transpose();
            
                pieceIdx.array() += 1;
            
                if ((pieceIdx.array() <= 0).any())
                {
                    return false;
                }
            }
            
            pieceN = pieceIdx.sum();
            
            if (pieceN <= 0)
            {
                return false;
            }

            temporalDim = pieceN;
            spatialDim = 0;
            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1)
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    hPolyIdx(j) = i;
                }
            }

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVA, tailPVA, pieceN);
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));

            // Allocate temp variables
            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(6 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        inline double optimize(Trajectory<5> &traj,
                               const double &relCostTol)
        {
            optimizedStateValid = false;
            optimizedCost = INFINITY;

            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            minco.setParameters(points, times);
            Trajectory<5> initialTrajectory;
            minco.getTrajectory(initialTrajectory);
            initialCorridorDiagnostics =
                evaluateCorridorDiagnostics(initialTrajectory);
            finalCorridorDiagnostics = CorridorDiagnostics();
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_PolytopeSFC::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);

                finalCorridorDiagnostics =
                    evaluateCorridorDiagnostics(traj);

                // Save the converged optimization state.  Sensitivity
                // analysis later starts from exactly this x*.
                optimizedX = x;
                optimizedPoints = points;
                optimizedTimes = times;
                optimizedCost = minCostFunctional;
                optimizedStateValid = true;
            }
            else
            {
                optimizedStateValid = false;

                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }

        inline double continueOptimizeWithCorridorPenaltyScale(
            Trajectory<5> &traj,
            const double &relCostTol,
            const double corridorPenaltyScale)
        {
            if (!optimizedStateValid ||
                optimizedX.size() !=
                    temporalDim + spatialDim ||
                !optimizedX.allFinite() ||
                basePenaltyWt.size() !=
                    penaltyWt.size() ||
                basePenaltyWt.size() <= 0 ||
                !std::isfinite(
                    corridorPenaltyScale) ||
                corridorPenaltyScale <= 0.0)
            {
                traj.clear();
                return INFINITY;
            }

            // --------------------------------------------------------
            // Preserve the converged state so a failed continuation
            // cannot destroy the previous valid solution.
            // --------------------------------------------------------
            const Eigen::VectorXd
                previousOptimizedX =
                    optimizedX;

            const Eigen::Matrix3Xd
                previousOptimizedPoints =
                    optimizedPoints;

            const Eigen::VectorXd
                previousOptimizedTimes =
                    optimizedTimes;

            const double previousOptimizedCost =
                optimizedCost;

            const Eigen::VectorXd
                previousPenaltyWt =
                    penaltyWt;

            // --------------------------------------------------------
            // Scale ONLY the corridor-position penalty.
            //
            // All dynamic penalties remain identical to the original
            // planning problem.
            // --------------------------------------------------------
            penaltyWt =
                basePenaltyWt;

            penaltyWt(0) =
                basePenaltyWt(0) *
                corridorPenaltyScale;

            // --------------------------------------------------------
            // Critical difference from optimize():
            //
            //     DO NOT call setInitial().
            //
            // Start directly from the converged optimization variable
            // x* from the previous solve.
            // --------------------------------------------------------
            Eigen::VectorXd x =
                optimizedX;

            Eigen::Map<Eigen::VectorXd>
                tau(
                    x.data(),
                    temporalDim);

            Eigen::Map<Eigen::VectorXd>
                xi(
                    x.data() +
                        temporalDim,
                    spatialDim);

            // Reconstruct the Cartesian MINCO state represented by x*.
            forwardT(
                tau,
                times);

            forwardP(
                xi,
                vPolyIdx,
                vPolytopes,
                points);

            minco.setParameters(
                points,
                times);

            Trajectory<5>
                continuationInitialTrajectory;

            minco.getTrajectory(
                continuationInitialTrajectory);

            initialCorridorDiagnostics =
                evaluateCorridorDiagnostics(
                    continuationInitialTrajectory);

            finalCorridorDiagnostics =
                CorridorDiagnostics();

            double minCostFunctional =
                INFINITY;

            lbfgs_params.mem_size =
                256;

            lbfgs_params.past =
                3;

            lbfgs_params.min_step =
                1.0e-32;

            lbfgs_params.g_epsilon =
                0.0;

            lbfgs_params.delta =
                relCostTol;

            const int ret =
                lbfgs::lbfgs_optimize(
                    x,
                    minCostFunctional,
                    &GCOPTER_PolytopeSFC::
                        costFunctional,
                    nullptr,
                    nullptr,
                    this,
                    lbfgs_params);

            if (ret >= 0)
            {
                forwardT(
                    tau,
                    times);

                forwardP(
                    xi,
                    vPolyIdx,
                    vPolytopes,
                    points);

                minco.setParameters(
                    points,
                    times);

                minco.getTrajectory(
                    traj);

                finalCorridorDiagnostics =
                    evaluateCorridorDiagnostics(
                        traj);

                optimizedX =
                    x;

                optimizedPoints =
                    points;

                optimizedTimes =
                    times;

                optimizedCost =
                    minCostFunctional;

                optimizedStateValid =
                    true;

                return minCostFunctional;
            }

            // --------------------------------------------------------
            // Continuation failed:
            // restore the previous converged optimization state.
            // --------------------------------------------------------
            optimizedX =
                previousOptimizedX;

            optimizedPoints =
                previousOptimizedPoints;

            optimizedTimes =
                previousOptimizedTimes;

            optimizedCost =
                previousOptimizedCost;

            optimizedStateValid =
                true;

            penaltyWt =
                previousPenaltyWt;

            minco.setParameters(
                optimizedPoints,
                optimizedTimes);

            traj.clear();

            return INFINITY;
        }

        inline bool computeDeformationMetrics(
            DeformationMetrics &metrics,
            const double displacementStep = 0.02,
            const double jacobianDamping = 1.0e-6,
            const double maxAnisotropy = 10.0,
            const DeformationMetricObjective objectiveMode =
                DeformationMetricObjective::FULL)
        {
            metrics.clear();

            if (!optimizedStateValid ||
                pieceN <= 0)
            {
                return false;
            }

            metrics.resize(pieceN);

            bool anyValid = false;

            for (int pieceId = 0;
                 pieceId < pieceN;
                 ++pieceId)
            {
                const bool success =
                    computeSingleDeformationMetric(
                        pieceId,
                        metrics[pieceId],
                        displacementStep,
                        jacobianDamping,
                        maxAnisotropy,
                        objectiveMode);

                anyValid =
                    anyValid || success;
            }

            // evaluateObjectiveAt() changes temporary GCOPTER buffers.
            // Restore the nominal optimized state before returning.
            Eigen::VectorXd restoreGradient;

            evaluateObjectiveAt(
                optimizedX,
                restoreGradient);

            return anyValid;
        }
            
        inline bool setGaussNewtonReferenceState(
            const Eigen::Matrix3d &referenceHeadPVA,
            const Eigen::Matrix3d &referenceTailPVA,
            const Eigen::Matrix3Xd &referencePoints,
            const Eigen::VectorXd &referenceTimes,
            const int integrationResolution,
            const Eigen::VectorXd &magnitudeBounds,
            const Eigen::VectorXd &physicalParams)
        {
            optimizedStateValid =
                false;
        
            const int referencePieceCount =
                referenceTimes.size();
        
            if (referencePieceCount <= 0 ||
                referencePoints.rows() != 3 ||
                referencePoints.cols() !=
                    referencePieceCount - 1 ||
                integrationResolution <= 0 ||
                magnitudeBounds.size() < 5 ||
                physicalParams.size() < 6 ||
                !referenceHeadPVA.allFinite() ||
                !referenceTailPVA.allFinite() ||
                !referencePoints.allFinite() ||
                !referenceTimes.allFinite() ||
                !magnitudeBounds.allFinite() ||
                !physicalParams.allFinite())
            {
                return false;
            }
        
            for (int pieceId = 0;
                 pieceId <
                     referencePieceCount;
                 ++pieceId)
            {
                if (!std::isfinite(
                        referenceTimes(pieceId)) ||
                    referenceTimes(pieceId) <=
                        0.0)
                {
                    return false;
                }
            }
        
            headPVA =
                referenceHeadPVA;
        
            tailPVA =
                referenceTailPVA;
        
            pieceN =
                referencePieceCount;
        
            integralRes =
                integrationResolution;
        
            magnitudeBd =
                magnitudeBounds;
        
            physicalPm =
                physicalParams;
        
            optimizedPoints =
                referencePoints;
        
            optimizedTimes =
                referenceTimes;
        
            optimizedCost =
                0.0;
        
            optimizedX.resize(0);
        
            minco.setConditions(
                headPVA,
                tailPVA,
                pieceN);
            
            minco.setParameters(
                optimizedPoints,
                optimizedTimes);
            
            flatmap.reset(
                physicalPm(0),
                physicalPm(1),
                physicalPm(2),
                physicalPm(3),
                physicalPm(4),
                physicalPm(5));
            
            optimizedStateValid =
                true;
            
            return true;
        }

        inline bool computeGaussNewtonDeformationMetrics(
            GaussNewtonDeformationMetrics &metrics,
            const double displacementStep = 0.01,
            const double relativeDamping = 1.0e-3,
            const double proximityPower = 4.0,
            const double maxCorridorAnisotropy = 10.0)
        {
            metrics.clear();
        
            if (!optimizedStateValid ||
                pieceN <= 0)
            {
                return false;
            }
        
            Eigen::Matrix<
                double,
                4,
                Eigen::Dynamic>
                nodeWeights;
        
            Eigen::Vector4d meanRatios;
            Eigen::Vector4d maxRatios;
            Eigen::Vector4d meanWeights;
        
            if (!evaluateNominalDynamicsProximityWeights(
                    nodeWeights,
                    meanRatios,
                    maxRatios,
                    meanWeights,
                    proximityPower))
            {
                return false;
            }
        
            metrics.resize(
                pieceN);
            
            bool allValid =
                true;
            
            for (int pieceId = 0;
                 pieceId < pieceN;
                 ++pieceId)
            {
                const bool success =
                    computeSingleGaussNewtonDeformationMetric(
                        pieceId,
                        metrics[pieceId],
                        displacementStep,
                        relativeDamping,
                        proximityPower,
                        maxCorridorAnisotropy,
                        nodeWeights,
                        meanRatios,
                        maxRatios,
                        meanWeights);
                    
                allValid =
                    allValid &&
                    success;
            }
        
            minco.setParameters(
                optimizedPoints,
                optimizedTimes);
            
            return allValid;
        }

        inline bool hasOptimizedState() const
        {
            return optimizedStateValid;
        }

        inline const Eigen::VectorXd &
        getOptimizedX() const
        {
            return optimizedX;
        }

        inline const Eigen::Matrix3Xd &
        getOptimizedPoints() const
        {
            return optimizedPoints;
        }

        inline const Eigen::VectorXd &
        getOptimizedTimes() const
        {
            return optimizedTimes;
        }

        inline double getOptimizedCost() const
        {
            return optimizedCost;
        }

        inline int getPieceNum() const
        {
            return pieceN;
        }

        inline const CorridorDiagnostics &getInitialCorridorDiagnostics() const
        {
            return initialCorridorDiagnostics;
        }

        inline const CorridorDiagnostics &getFinalCorridorDiagnostics() const
        {
            return finalCorridorDiagnostics;
        }
    };

}

#endif
