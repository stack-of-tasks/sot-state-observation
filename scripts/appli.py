# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

appli = CompensaterApplication(robot)
appli.withTraces()

if 'usingRobotViewer' in locals() and usingRobotViewer:    refreshList.append(lambda: appli.updateDisplay());    appli.initDisplay(); go()

# --- SHORTCUTS
pop          = appli.pop
push         = appli.push
dyn          = appli.robot.dynamic
sot          = appli.sot
contactLF    = appli.contactLF
contactRF    = appli.contactRF
taskLim      = appli.taskLim
taskCom      = appli.taskCom
taskChest    = appli.taskChest
taskGaze     = appli.taskGaze
taskPosture  = appli.taskPosture
taskRH       = appli.taskRH
taskR        = appli.taskCompensate
tr           = appli.robot.tracer
gopen        = optionalparentheses(appli.openGripper)
gclose       = optionalparentheses(appli.closeGripper)

t = optionalparentheses(appli.trace)
a = appli
s = optionalparentheses(appli.nextStep)

#stop()

