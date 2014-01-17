INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
#  FILES ros/sot-concept.py
#  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

INSTALL(FILES
  scripts/stabilizer/compensater.py
  scripts/stabilizer/__init__.py 
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/stabilizer
)

