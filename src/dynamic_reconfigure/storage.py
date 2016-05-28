# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospkg
import rospy

__all__ = ['BaseStorage', 'load_plugin', 'load_plugins']


class BaseStorage(object):

    def __init__(self, node_name, storage_url):
        pass

    def load_config(self, msg):
        raise NotImplementedError()

    def save_config(self, msg):
        raise NotImplementedError()


def load_plugin(plugin):
    rospack = rospkg.RosPack()
    pkg_list = rospack.get_depends_on('dynamic_reconfigure', implicit=False)

    for pkg in pkg_list:
        m = rospack.get_manifest(pkg)
        p_modules = m.get_export('dynamic_reconfigure', 'plugin_py')
        if not p_modules:
            continue

        if plugin in p_modules:
            try:
                # import the specified plugin module
                (p_module_0, p_klass) = plugin.rsplit('.', 1)
                mod = __import__(p_module_0)
                for sub_mod in p_module_0.split('.')[1:]:
                    mod = getattr(mod, sub_mod)

                klass = getattr(mod, p_klass)

                if not issubclass(klass, BaseStorage):
                    rospy.logerr("Cannot load plugin [%s]: plugin must inherit `dynamic_reconfigure.storage.BaseStorage` class.", plugin)
                    continue

                return klass

            except Exception as ex:
                rospy.logerr("Cannot load plugin [%s]: %s", plugin, str(ex))

    return None


def load_plugins():
    rospack = rospkg.RosPack()
    pkg_list = rospack.get_depends_on('dynamic_reconfigure', implicit=False)

    for pkg in pkg_list:
        m = rospack.get_manifest(pkg)
        p_modules = m.get_export('dynamic_reconfigure', 'plugin_py')
        if not p_modules:
            continue

        for p_module in p_modules:
            try:
                # import the specified plugin module
                (p_module_0, p_klass) = p_module.rsplit('.', 1)
                mod = __import__(p_module_0)
                for sub_mod in p_module_0.split('.')[1:]:
                    mod = getattr(mod, sub_mod)

                klass = getattr(mod, p_klass)

                if not issubclass(klass, BaseStorage):
                    rospy.logerr("Cannot load plugin [%s]: plugin must inherit `dynamic_reconfigure.storage.BaseStorage` class.", p_module)
                    continue

                yield (p_module, klass)

            except Exception as ex:
                rospy.logerr("Cannot load plugin [%s] from package [%s]: %s", p_module, pkg, str(ex))
