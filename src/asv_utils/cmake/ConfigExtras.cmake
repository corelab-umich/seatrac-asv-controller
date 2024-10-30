# amend_export_dependencies doesn't support COMPONENTS (yet) so we have to separate 
# out Boost into its own ConfigExtras file so its properly found by downstream client packages
find_package(Boost 1.35.0 REQUIRED COMPONENTS system) # only boost versions >=1.35.0 come bundled with the asio networking library