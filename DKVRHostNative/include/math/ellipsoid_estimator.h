#pragma once

#include <vector>

#include "Eigen/Core"

namespace dkvr {

	/// <summary>
	/// <para>Represents the quadratic equation of ellipsoid.</para>
	/// <para>x^Ax + b^x + d = 0, which operator(^) denotes transpose.</para>
	/// </summary>
	class EllipsoidParameter
	{
	public:
		EllipsoidParameter(const Eigen::Matrix3f& a, const Eigen::Vector3f& b, float d) : a(a), b(b), d(d) {}

		/// <summary>
		/// <para>Returns a Vector representing the center of the ellipsoid.</para>
		/// <para>(x-c)^A`(x-c) = 1, which operator(^) denotes transpose.</para>
		/// <para>Beaware that matrix A` in the above equation differs from matirx A in the equation of class summary.</para>
		/// <para>The center vector is calculated as below:</para>
		/// <para>c = -0.5 * inv(A) * b</para>
		/// </summary>
		/// <returns>A vector 'c' representing the center of the ellipsoid</returns>
		Eigen::Vector3f GetCenterVector() const;

		/// <summary>
		/// <para>Calculates the transformation matrix that transforms a unit sphere into the ellipsoid.</para>
		/// <para>y = Tx + c, where x belongs to a unit sphere and y belongs to the ellipsoid.</para>
		/// <para>Beaware that this transformation matrix does not contain translation component.</para>
		/// <para>The transformation matrix is calculated by applying SVD on A` as below:</para>
		/// <para>A` = U^¥ËU, which ¥Ë is eigenvalue diagonal matrix and U is eigenvector matrix.</para>
		/// <para>T = U^¥ÒU, which ¥Ò is sqaure root of ¥Ë.</para>
		/// </summary>
		/// <returns>A matrix T representing the transformation matrix</returns>
		Eigen::Matrix3f GetTransformationMatrix() const;

		Eigen::Matrix3f a;
		Eigen::Vector3f b;
		float d;
	};

	class EllipsoidEstimator
	{
	public:
		/// <summary>
		/// <para>Estimate the ellipsoid parameter by using Adjusted Least Squares method.</para>
		/// <para>ref: "Consistent Least Squares Fitting of Ellipsoids." Numerische Mathematik 98 (2004): 177-194.</para>
		/// </summary>
		/// <returns>A EllipsoidParameter representing the ellipsoid best fitting to samples.</returns>
		static EllipsoidParameter EstimateEllipsoid(const std::vector<Eigen::Vector3f>& samples, float noise_var);
	};

}	// namespace dkvr