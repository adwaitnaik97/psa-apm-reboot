@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/"$AIOS_ID"_aios_config.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/aios_config/"$AIOS_ID"_aios_config.bash"
@[end if]@
