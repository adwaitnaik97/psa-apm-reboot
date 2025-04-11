@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/virtualenv_source.sh"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/virtualenv_config/virtualenv_source.sh"
@[end if]@
