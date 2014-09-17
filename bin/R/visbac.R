library(ggplot2)
library(grid)
library(RColorBrewer)
library(XML)
library(scales)

circleFun <- function(p = c(0,0), r = 1, npoints = 100){
  t = seq(0, 2*pi, length.out = npoints)
  xx = p[1] + r * cos(t)
  yy = p[2] + r * sin(t)
  return(data.frame(x = xx, y = yy, r=r))
}

dir = "~/Experiments/Bacterium/Evo/SensorAbs/Rand/"
foldersNoReLearn = c("14_06_04__18_40_41", "14_06_04__17_08_45", "14_06_04__17_08_53",
            "14_06_04__19_15_17", "14_06_04__19_15_19", "14_06_20__09_13_36",
            "14_06_24__12_10_01")

foldersReLearn = c("14_06_27__12_10_18", "14_06_27__09_48_58", "14_06_27__11_58_09",
                   "14_07_03__18_42_42", "14_07_11__13_33_03")


folder = foldersReLearn[[5]]

file = paste(dir, folder, "/State.txt", sep="")
cfgfnm = paste(dir, folder, "/Bacterium.xml", sep="")
evofnm = paste(dir, folder, "/Evolvable.xml", sep="")
cfg = xmlInternalTreeParse(cfgfnm)
evo = xmlInternalTreeParse(evofnm)

numTe = strtoi(xpathApply(cfg, "/Config/GA/Evolvable/Trial", xmlGetAttr, "numTests")[[1]])
numEnv = strtoi(xpathApply(cfg, "/Config/GA/Evolvable/Trial", xmlGetAttr, "numEnvirons")[[1]])
numPh = numTe * numEnv
numTr = strtoi(xpathApply(cfg, "/Config/GA/Eval/NumTrials", xmlValue)[[1]])
totalPh = numTr * numPh

stageEvo = strtoi(xpathApply(evo, "/Evolvable/Genome", xmlGetAttr, "Stage")[[1]])
stageTest = strtoi(xpathApply(cfg, "/Config/GA/Eval", xmlGetAttr, "Stage")[[1]])
fit = as.numeric(xpathApply(evo, "/Evolvable/Genome", xmlGetAttr, "Fitness")[[1]])

D = read.table(file, header=T)
n = nrow(D)
phlength = n / totalPh

times = c(0:(n-1))
D$phase = times %/% phlength
D$time = times %% phlength

# remove last step in each trial
D = D[D$time!=(phlength-1),]
D$nph = D$phase %% numPh
D$tr = D$phase %/% numPh
n = nrow(D)
phlength = n / totalPh

cols = colorRampPalette(brewer.pal(9,"Greys"))(200)

# Create circles
cs = data.frame()
for (i in c(0:(totalPh-1))){
  #isOddTr = (i%/%numPh) %% 2
  #r = 0.3 + isOddTr*0.5
  #r = unique(D$foodR)[[i+1]]
  r = D$foodR[[i*phlength+1]]
  c = circleFun(c(0,0), r)
  c$phase = i
  if ((i %% 3) == 2)
    c$cola = "white"
  else
    c$cola = "green"
  cs = rbind(cs, c)
}

# Main Euclidean trajectory plot
trajs = ggplot(D, aes(x = PosX, y = PosY)) + 
  geom_path(data=cs, aes(x,y), color=alpha("green", 0.3), size=3) +
  geom_path(aes(color = SensedEnergy)) +
  coord_fixed() +
  theme_classic() + 
  scale_color_gradient(limits=c(0, 1), low="red", high="blue", guide=F) +
  #scale_color_gradientn(colours=rainbow(7), guide=F) +
  facet_wrap(~phase, ncol=numPh) +
  ggtitle(sprintf("EvoStage=%i F=%1.2f TestStage=%i\n", stageEvo, fit, stageTest))

print(trajs)

# Print neural trajectories
varname = "NeuralOutput"
varnums = c(0:8)
vars = c("Time", "Distance", paste(varname, varnums, sep=""))

trial = 0
phase = 5

Dtr = D[D$tr==trial & D$nph==phase, vars]
Dm = melt(Dtr, id.vars="Time")

ggplot(Dm, aes(x = Time, y = value)) + 
  geom_line() +
  theme_minimal() +
  theme(panel.margin = unit(1.5, "line")) +
  #theme(axis.ticks = element_blank(), axis.text.y = element_blank()) +
  #scale_y_continuous(breaks=NULL) +
  #scale_y_continuous(breaks=seq(-1,1,1)) +
  facet_grid(variable ~ ., scales="free")
  
