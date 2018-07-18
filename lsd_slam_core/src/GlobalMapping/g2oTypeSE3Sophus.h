/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "util/SophusUtil.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


namespace lsd_slam
{


class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexSE3();
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl() {
		_estimate = Sophus::SE3d();
	}

	virtual void oplusImpl(const double* update_)
	{
		Eigen::Map< Eigen::Matrix<double, 6, 1> > update(const_cast<double*>(update_));

		//if (_fix_scale) update[6] = 0;

		setEstimate(Sophus::SE3d::exp(update) * estimate());
	}

	bool _fix_scale;
};

/**
* \brief 6D edge between two Vertex6
*/
class EdgeSE3 : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3, VertexSE3>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3();
	
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
	
	void computeError()
	{
		const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);
		const VertexSE3* _to = static_cast<const VertexSE3*>(_vertices[1]);

		//Compute difference between estimates, then effectively subtract the constraint
		Sophus::SE3d error_= _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;
		_error = error_.log();

	}
	
	void linearizeOplus()
	{
		const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);

		_jacobianOplusXj = _from->estimate().inverse().Adj();
		_jacobianOplusXi = -_jacobianOplusXj;
	}


	virtual void setMeasurement(const Sophus::SE3d& m)
	{
		_measurement = m;
		_inverseMeasurement = m.inverse();
	}
	
	virtual bool setMeasurementData(const double* m)
	{
		Eigen::Map<const Sophus::Vector6d> v(m);
		setMeasurement(Sophus::SE3d::exp(v));
		return true;
	}
	
	virtual bool setMeasurementFromState()
	{
		const VertexSE3* from = static_cast<const VertexSE3*>(_vertices[0]);
		const VertexSE3* to   = static_cast<const VertexSE3*>(_vertices[1]);
		Sophus::SE3d delta = from->estimate().inverse() * to->estimate();
		setMeasurement(delta);
		return true;
	}

	virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
	
	virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
	{
		VertexSE3 *_from = static_cast<VertexSE3*>(_vertices[0]);
		VertexSE3 *_to   = static_cast<VertexSE3*>(_vertices[1]);

		if (from.count(_from) > 0)
			_to->setEstimate(_from->estimate() * _measurement);
		else
			_from->setEstimate(_to->estimate() * _inverseMeasurement);
	}
	
protected:
	Sophus::SE3d _inverseMeasurement;
};

//Packaged class to store SE(3) without x axis
//Internally we store as SE(3)
class SE3NoX {
	public:
		SE3NoX()
		{
			_transform = Sophus::SE3d();
		}

		SE3NoX(Eigen::Matrix3d &rot, Eigen::Vector2d &trans) 
		{
			_transform.setRotationMatrix(rot);
            _transform.translation() = Eigen::Vector3d(trans[0], 0, trans[1]);
		}

		SE3NoX(Eigen::Quaterniond &quat, Eigen::Vector2d &trans) 
		{
			_transform.setQuaternion(quat);
			_transform.translation() = Eigen::Vector3d(trans[0], 0, trans[1]);
		}

		SE3NoX(double roll, double pitch, double yaw, double y, double z) 
		{
            _transform.translation() = Eigen::Vector3d(y,0,z);
            _transform.setQuaternion(rpyToQuat(roll,pitch,yaw));
		}

		SE3NoX(const Sophus::SE3d &trans)
		{
			_transform = trans;
			_transform.translation()(1) = 0;
		}

        Sophus::SE3d& transform()
        {
            return _transform;
        }

	private:
		Sophus::SE3d _transform;

        Eigen::Quaterniond rpyToQuat(double r, double p, double y)
        {
            Eigen::Quaterniond q;
            q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitZ());
            return q;
        }
};

/**
* \brief 5D edge between two Vertex6 (do not constrain x-axis motion)
* Still 6d because we still are internally just on SE3
*/
class EdgeSE3NoX : public g2o::BaseBinaryEdge<6, SE3NoX, VertexSE3, VertexSE3>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3NoX();
	
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
	
	void computeError()
	{
		const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);
		const VertexSE3* _to = static_cast<const VertexSE3*>(_vertices[1]);

		SE3NoX error_(_from->estimate().inverse() * _to->estimate() * _inverseMeasurement);
		_error = error_.transform().log();
	}
	
	void linearizeOplus()
	{
		const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);

		_jacobianOplusXj = _from->estimate().inverse().Adj();
		_jacobianOplusXi = -_jacobianOplusXj;
	}


	virtual void setMeasurement(const SE3NoX& m)
	{
		_measurement = m;
		_inverseMeasurement = _measurement.transform().inverse();
	}
	
	virtual bool setMeasurementData(const double* m)
	{
		Eigen::Map<const Sophus::Vector6d> v(m);
		setMeasurement(SE3NoX(Sophus::SE3d::exp(v)));
		return true;
	}
	
	virtual bool setMeasurementFromState()
	{
		const VertexSE3* from = static_cast<const VertexSE3*>(_vertices[0]);
		const VertexSE3* to   = static_cast<const VertexSE3*>(_vertices[1]);
		Sophus::SE3d delta = from->estimate().inverse() * to->estimate();
		setMeasurement(SE3NoX(delta));
		return true;
	}

	virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
	
	virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
	{
		VertexSE3 *_from = static_cast<VertexSE3*>(_vertices[0]);
		VertexSE3 *_to   = static_cast<VertexSE3*>(_vertices[1]);

		if (from.count(_from) > 0)
			_to->setEstimate(_from->estimate() * _measurement.transform());
		else
			_from->setEstimate(_to->estimate() * _inverseMeasurement);
	}
private:
	Sophus::SE3d _inverseMeasurement;
};

} //End namespace
