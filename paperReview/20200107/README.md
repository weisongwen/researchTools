# VISUAL-INERTIAL NAVIGATION: A CONCISE REVIEW([pdf](https://arxiv.org/pdf/1906.02650.pdf))

## Abstract
As inertial and visual sensors are becoming ubiquitous, visual-inertial navigation systems (VINS)
have prevailed in a wide range of applications from mobile augmented reality to aerial navigation to
autonomous driving, in part because of the complementary sensing capabilities and the decreasing
costs and size of the sensors. In this paper, we survey thoroughly the research efforts taken in this
field and strive to provide a concise but complete review of the related work – which is unfortunately
missing in the literature while being greatly demanded by researchers and engineers – in the hope to
accelerate the VINS research and beyond in our society as a whole.

**Authors**
Guoquan (Paul) Huang
Robot Perception and Navigation Group
University of Delaware, Newark, DE 19716
Email: ghuang@udel.edu

<p align="center">
  <img width="502pix" src="framework.png">
</p>


## comments
 Discussions and Conclusions
As inertial and visual sensors are becoming ubiquitous, visual-inertial navigation systems (VINS) have incurred significant research efforts and witnessed great progresses in the past decade, fostering an increasing number of innovative
applications in practice. As a special instance of the well-known SLAM problem, VINS researchers have been quickly
building up a rich body of literature on top of SLAM [36]. Given the growing number of papers published in this field,
it has become harder (especially for practitioners) to keep up with the state of the art. Moreover, because of the particular sensor characteristics, it is not trivial to develop VINS algorithms from scratch without understanding the pros
and cons of existing approaches in the literature (by noting that each method has its own particular focus and does not
necessarily explain all the aspects of VINS estimation). All these have motivated us to provide this review on VINS,
which, to the best of our knowledge, is unfortunately lacked in the literature and thus should be a useful reference
for researchers/engineers who are working on this problem. Upon our significant prior work in this domain, we have
strived to make this review concise but complete, by focusing on the key aspects about building a VINS algorithm
including state estimation, sensor calibration and observability analysis.
While there are significant progresses on VINS made in the past decade, many challenges remain to cope with, and in
the following we just list a few open to discuss:
• Persistent localization: While current VINS are able to provide accurate 3D motion tracking, but, in smallscale friendly environments, they are not robust enough for long-term, large-scale, safety-critical deployments, e.g., autonomous driving, in part due to resource constraints [95, 97, 164]. As such, it is demanding to
enable persistent VINS even in challenging conditions (such as bad lighting and motions), e.g., by efficiently
integrating loop closures or building and utilizing novel maps.
• Semantic localization and mapping: Although geometric features such as points, lines and planes [151, 165]
are primarily used in current VINS for localization, these handcrafted features may not be work best for
navigation, and it is of importance to be able to learn best features for VINS by leveraging recent advances
of deep learning [166]. Moreover, a few recent research efforts have attempted to endow VINS with semantic understanding of environments [167, 168, 169, 170], which is only sparsely explored but holds great
potentials.
• High-dimensional object tracking: When navigating in dynamic complex environments, besides highprecision localization, it is often necessary to detect, represent, and track moving objects that co-exist in
the same space in real time, for example, 3D object tracking in autonomous navigation [92, 171, 172].
• Distributed cooperative VINS: Although cooperative VINS have been preliminarily studied in [126, 173], it
is still challenging to develop real-time distributed VINS, e.g., for crowd sourcing operations. Recent work
on cooperative mapping [174, 175] may shed some light on how to tackle this problem.
• Extensions to different aiding sensors: While optical cameras are seen an ideal aiding source for INS in
many applications, other aiding sensors may more proper for some environments and motions, for example,
acoustic sonars may be instead used in underwater [176]; low-cost light-weight LiDARs may work better in
environments, e.g., with poor lighting conditions [71, 177]; and event cameras [178, 179] may better capture
dynamic motions [180, 181]. Along this direction, we should investigate in-depth VINS extensions of using
different aiding sources for applications at hand.

**Advantage**: This paper extensively review the VINS. The paper writing is pretty fluent.
**Question**: 
- how the future of cooperative mapping should be?

