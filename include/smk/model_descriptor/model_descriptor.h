#ifndef SMK_MODEL_DESCRIPTOR_MODEL_DESCRIPTOR_H_
#define SMK_MODEL_DESCRIPTOR_MODEL_DESCRIPTOR_H_

#include <Eigen/Dense>
#include <memory>
#include <list>
#include <string>

namespace smk
{
    namespace model_descriptor
    {
        /// \brief Rigid frame for each robot part.
        ///
        class Frame
        {
        public:
            /// \brief Construct a new Frame
            ///
            Frame();
            /// \brief Destroy the Frame
            ///
            virtual ~Frame();
            /// \brief Get Frame ID. You CANNOT set this value manually.
            /// Everytime you create Frame instance, the value of id increases by 1.
            /// This id guarantee uniqueness unless you create more than INTMAX number of Frames.
            /// \return const int& id  ID of this frame.
            const int &GetID() const;
            /// \brief Set the Name of this frame.
            ///
            /// \param[in] name Name of this Frame
            void SetName(const std::string &name);
            /// \brief Get the Name of this frame.
            ///
            /// \return const std::string& Name of this frame.
            const std::string &GetName() const;
            /// \brief Set the Parent frame.
            ///
            /// \param[in] frame parent frame.
            void SetParent(std::shared_ptr<Frame> &frame);
            /// \brief Get the Parent frame
            ///
            /// \return std::shared_ptr<Frame> parent frame.
            std::shared_ptr<Frame> GetParent() const;
            /// \brief Add child frame to the children list.
            ///
            /// \param[in] child child frame.
            void AddChild(std::shared_ptr<Frame> child);
            /// \brief Get the Children list
            ///
            /// \return std::list<std::weak_ptr<Frame>>& list of children frames.
            std::list<std::weak_ptr<Frame>> &GetChildren() const;
            /// \brief Set the Origin of this frame.
            ///
            /// \param[in] mat origin in the 3d affine transformation.
            void SetOrigin(const Eigen::Affine3d &mat);
            /// \brief Set the Origin of this frame.
            ///
            /// \param[in] trans translation vector in 3D.
            /// \param[in] rot  Angle-Axis rotation
            void SetOrigin(const Eigen::Vector3d &trans, const Eigen::AngleAxisd &rot);
            /// \brief Get the Origin of this frame.
            ///
            /// \return const Eigen::Affine3d&  3D Affine transformation matrix of the origin.
            const Eigen::Affine3d &GetOrigin() const;
            /// \brief Set the Transformation from the parent frame.
            ///
            /// \param[in] mat origin in the 3d affine transformation.
            void SetTransform(const Eigen::Affine3d &mat);
            /// \brief Set the Transformation from the parent frame.
            ///
            /// \param[in] trans translation vector in 3D.
            /// \param[in] rot  Angle-Axis rotation
            void SetTransform(const Eigen::Vector3d &trans, const Eigen::AngleAxisd &rot);
            /// \brief Get the Transformation from parent frame.
            ///
            /// \return const Eigen::Affine3d& transformation matrix
            const Eigen::Affine3d &GetTransform() const;

            /// \brief Update transformation from the origin.
            /// (this->transformation) = inv(parent-> transformation) * (this->origin)
            ///
            void UpdateTransformationFromOrigin();

        private:
            struct impl;
            std::unique_ptr<impl> data_;
        };

        class Link : public Frame
        {
        public:
            Link();
            ~Link();
            // Reserved for the features for link geometry and dynamic properties.
        private:
        };

        /// \brief Joint of the manipulator.
        ///
        class Joint : public Frame
        {
        public:
            /// \brief Construct a new Joint
            ///
            Joint();
            /// \brief Destroy the Joint
            ///
            ~Joint();
            /// \brief Set the Axis vector.
            /// your input vector will be normalized inside this function.
            ///
            /// \param[in] axis A 3D vector representing the axis from the origin of this frame.
            void SetAxis(const Eigen::Vector3d &axis);
            /// \brief Get the Axis
            ///
            /// \return const Eigen::Vector3d& normalized vector of axis in this frame's coordinate.
            const Eigen::Vector3d &GetAxis() const;
            /// \brief Set the Angle Limit
            ///
            /// \param[in] upper_rad upper angle limmit in radian.
            /// \param[in] lower_rad lower angle limit in radian.
            void SetAngleLimit(const double &upper_rad, const double &lower_rad);
            /// \brief Check that the input angle does not violate the limits.
            ///
            /// \param[in] rad input angle
            /// \return true angle is between upper limit and lower limmit
            /// \return false angle is greater than the upper limit or less than the lower limit.
            bool IsValidAngle(const double &rad) const;

        private:
            struct impl;
            std::unique_ptr<impl> data_;
        };

    } // namespace model_descriptor
} // namespace smk
#endif // MODEL_DESCRIPTOR_MODEL_DESCRIPTOR_H_
