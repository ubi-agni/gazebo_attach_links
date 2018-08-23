# Overview

Gazebo world plugin to virtually attach a _reference link_ of a _reference model_ to a free-floating _target link_ of a second _target model_.
At each time step, the _target link_ is enforced to a pose relative to the _reference_link_. There is no physical interaction between the two links (no inertia, mass change of any link)

# Usage

The plugin must be added to your world to permit operations on any subsequently loaded models.

The plugin takes commands over a topic _/gazebo_attach_  via queries of an _operation_ to apply on given parameters. The current attached state is published once every 1000 simulation steps to _/gazebo_attached_

