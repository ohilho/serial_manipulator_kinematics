#include "smk/model_descriptor/model_descriptor.h"

namespace smk
{
    namespace model_descriptor
    {
        struct Frame::impl
        {
            std::string name_;
            std::weak_ptr<Frame> parent_;
            std::list<std::weak_ptr<Frame>> children_;
            Eigen::Affine3d org_;
            Eigen::Affine3d tf_;
            int id_;
            static int count_;
        };

        int Frame::impl::count_ = 0;

        Frame::Frame() : data_(new impl)
        {
            data_->name_ = "anonymous";
            data_->org_.setIdentity();
            data_->tf_.setIdentity();
            data_->id_ = data_->count_;
            data_->count_++;
        }

        Frame::~Frame()
        {
        }

        const int &Frame::GetID() const
        {
            return data_->id_;
        }

        void Frame::SetName(const std::string &name)
        {
            data_->name_ = name;
        }

        const std::string &Frame::GetName() const
        {
            return data_->name_;
        }

        void Frame::SetParent(std::shared_ptr<Frame> &frame)
        {
            data_->parent_ = frame;
        }

        std::shared_ptr<Frame> Frame::GetParent() const
        {
            return data_->parent_.lock();
        }

        void Frame::AddChild(std::shared_ptr<Frame> child)
        {
            data_->children_.push_back(child);
        }

        std::list<std::weak_ptr<Frame>> &Frame::GetChildren() const
        {
            return data_->children_;
        }

        void Frame::SetOrigin(const Eigen::Affine3d &mat)
        {
            data_->org_ = mat;
        }

        void Frame::SetOrigin(const Eigen::Vector3d &trans, const Eigen::AngleAxisd &rot)
        {
            data_->org_.setIdentity();
            data_->org_.translate(trans).rotate(rot);
        }

        const Eigen::Affine3d &Frame::GetOrigin() const
        {
            return data_->org_;
        }
        void Frame::SetTransform(const Eigen::Affine3d &mat)
        {
            data_->tf_ = mat;
        }
        void Frame::SetTransform(const Eigen::Vector3d &trans, const Eigen::AngleAxisd &rot)
        {
            data_->tf_.setIdentity();
            data_->tf_.translate(trans).rotate(rot);
        }
        const Eigen::Affine3d &Frame::GetTransform() const
        {
            return data_->tf_;
        }
        void Frame::UpdateTransformationFromOrigin()
        {
            auto p = data_->parent_.lock();
            if (p)
            {
                data_->tf_ = p->GetOrigin().inverse() * GetOrigin();
            }
        }
        Link::Link() : Frame() {}
        Link::~Link() {}

        struct Joint::impl
        {
            Eigen::Vector3d axis_;
            double limits_[2];
        };

        Joint::Joint() : Frame(), data_(new impl)
        {
            // default Z axis
            data_->axis_ = Eigen::Vector3d(0.0, 0.0, 1.0);
        }

        Joint::~Joint()
        {
        }

        void Joint::SetAxis(const Eigen::Vector3d &axis)
        {
            data_->axis_ = axis;
        }

        const Eigen::Vector3d &Joint::GetAxis() const
        {
            return data_->axis_;
        }

        void Joint::SetAngleLimit(const double &upper_rad, const double &lower_rad)
        {
            data_->limits_[0] = lower_rad;
            data_->limits_[1] = upper_rad;
        }

        bool Joint::IsValidAngle(const double &rad) const
        {
            return (rad >= data_->limits_[0]) && (rad <= data_->limits_[1]);
        }

    }; // namespace model_descriptor

} // namespace smk
