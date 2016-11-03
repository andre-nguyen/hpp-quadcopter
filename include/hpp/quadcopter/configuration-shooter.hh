//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-quadcopter
// hpp-quadcopter is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-quadcopter is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-quadcopter  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_QUADCOPTER_BASIC_CONFIGURATION_SHOOTER_HH
# define HPP_QUADCOPTER_BASIC_CONFIGURATION_SHOOTER_HH

# include <hpp/core/configuration-shooter.hh>

# include <hpp/quadcopter/fwd.hh>
# include <hpp/quadcopter/config.hh>

namespace hpp {
  namespace quadcopter {
    /// \addtogroup configuration_sampling
    /// \{

    /// Uniformly sample with bounds of degrees of freedom.
    class HPP_QUADCOPTER_DLLAPI ConfigurationShooter :
      public core::ConfigurationShooter
    {
    public:
      static ConfigurationShooterPtr_t create (const DevicePtr_t& robot)
      {
	ConfigurationShooter* ptr = new ConfigurationShooter (robot);
	ConfigurationShooterPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      virtual ConfigurationPtr_t shoot () const;
    protected:
      /// Uniformly sample configuration space
      ///
      /// Note that translation joints have to be bounded.
      ConfigurationShooter (const DevicePtr_t& robot) : robot_ (robot)
      {
      }
      void init (const ConfigurationShooterPtr_t& self)
      {
        core::ConfigurationShooter::init (self);
	weak_ = self;
      }

    private:
      const DevicePtr_t& robot_;
      ConfigurationShooterWkPtr_t weak_;
    }; // class BasicConfigurationShooter
    /// \}
  } //   namespace quadcopter
} // namespace hpp

#endif // HPP_QUADCOPTER_BASIC_CONFIGURATION_SHOOTER_HH
