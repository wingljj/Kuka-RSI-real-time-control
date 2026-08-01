//
// rl_verify — verifies the RL 0.7.0 core build against the Comau Racer 7-1.4
// model: rlmdl XML load, forward kinematics, and inverse-kinematics round trip.
//
// Build steps and expected output are recorded in docs/rl-build-notes.md.
//

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <rl/math/Constants.h>
#include <rl/math/Transform.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

static void
printPose(const std::string& label, const rl::math::Transform& t)
{
	const Eigen::Vector3d p = t.translation();
	const Eigen::Quaterniond q(t.rotation());

	std::cout << label
		<< " position [mm]: " << std::fixed << std::setprecision(3)
		<< p.x() * 1000.0 << "  " << p.y() * 1000.0 << "  " << p.z() * 1000.0
		<< "  |  orientation (w x y z): " << std::setprecision(5)
		<< q.w() << "  " << q.x() << "  " << q.y() << "  " << q.z()
		<< std::endl;
}

int
main(int argc, char** argv)
{
	const char* path = argc > 1 ? argv[1]
		: "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml";

	try
	{
		// --- load rlmdl via rl::mdl::XmlFactory (libxml2-backed) ---
		rl::mdl::XmlFactory factory;
		std::shared_ptr<rl::mdl::Model> model = factory.create(path);
		std::shared_ptr<rl::mdl::Kinematic> kinematic =
			std::dynamic_pointer_cast<rl::mdl::Kinematic>(model);

		if (!kinematic)
		{
			throw std::runtime_error("model does not provide kinematics");
		}

		std::cout << "model: " << model->getManufacturer() << " "
			<< model->getName() << std::endl;
		std::cout << "dof: " << kinematic->getDof()
			<< ", joints: " << kinematic->getJoints()
			<< ", operational dof: " << kinematic->getOperationalDof()
			<< " (TCP frame '" << kinematic->getOperationalFrame(0)->getName()
			<< "')" << std::endl;

		// --- joint limits (model stores radians; XML was degrees) ---
		std::cout << "joint limits [deg]:" << std::endl;

		for (std::size_t i = 0; i < kinematic->getJoints(); ++i)
		{
			const rl::mdl::Joint* joint = kinematic->getJoint(i);
			std::cout << "  " << joint->getName() << ": ["
				<< joint->getMinimum()(0) * rl::math::constants::rad2deg << ", "
				<< joint->getMaximum()(0) * rl::math::constants::rad2deg << "]"
				<< std::endl;
		}

		// --- forward kinematics at the model home configuration ---
		rl::math::Vector qHome = kinematic->getHomePosition();
		std::cout << "home q [deg]: "
			<< (qHome * rl::math::constants::rad2deg).transpose() << std::endl;

		kinematic->setPosition(qHome);
		kinematic->forwardPosition();
		printPose("home TCP", kinematic->getOperationalPosition(0));

		// --- forward kinematics at q = 0 (all zeros) ---
		rl::math::Vector q0 = rl::math::Vector::Zero(kinematic->getDofPosition());
		kinematic->setPosition(q0);
		kinematic->forwardPosition();
		printPose("q=0  TCP", kinematic->getOperationalPosition(0));

		// --- IK round trip: goal = forward(home); start from a perturbed q ---
		kinematic->setPosition(qHome);
		kinematic->forwardPosition();
		const rl::math::Transform goal = kinematic->getOperationalPosition(0);

		rl::mdl::JacobianInverseKinematics ik(kinematic.get());
		ik.setDuration(std::chrono::milliseconds(500));
		ik.setEpsilon(1e-9);
		ik.addGoal(goal, 0);

		rl::math::Vector qStart = qHome;
		qStart(0) += 0.10;
		qStart(1) -= 0.20;
		qStart(2) += 0.15;
		qStart(4) -= 0.10;
		qStart(5) += 0.05;

		for (std::size_t i = 0; i < kinematic->getJoints(); ++i)
		{
			const rl::mdl::Joint* joint = kinematic->getJoint(i);
			qStart(i) = std::max(qStart(i), joint->getMinimum()(0));
			qStart(i) = std::min(qStart(i), joint->getMaximum()(0));
		}

		kinematic->setPosition(qStart);
		std::cout << "IK start q [deg]: "
			<< (qStart * rl::math::constants::rad2deg).transpose() << std::endl;

		const bool ok = ik.solve();
		kinematic->forwardPosition();

		const rl::math::Vector qSolved = kinematic->getPosition();
		std::cout << "IK solve: " << (ok ? "true" : "false") << std::endl;
		std::cout << "IK solved q [deg]: "
			<< (qSolved * rl::math::constants::rad2deg).transpose() << std::endl;

		const rl::math::Vector dq = qSolved - qHome;
		std::cout << "round-trip error per joint [rad]:" << std::endl;

		for (std::size_t i = 0; i < dq.size(); ++i)
		{
			std::cout << "  joint " << i << ": " << std::scientific
				<< std::setprecision(3) << std::abs(dq(i)) << std::endl;
		}

		const Eigen::Vector3d dp = kinematic->getOperationalPosition(0)
			.translation() - goal.translation();
		const Eigen::AngleAxisd da(goal.rotation().transpose()
			* kinematic->getOperationalPosition(0).rotation());
		std::cout << "pose residual: position [mm] " << std::fixed
			<< std::setprecision(6) << dp.norm() * 1000.0 << ", rotation [deg] "
			<< std::abs(da.angle()) * rl::math::constants::rad2deg << std::endl;

		return ok ? EXIT_SUCCESS : EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		std::cerr << "rl_verify: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}
