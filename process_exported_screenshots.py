import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from scipy import ndimage as ndi
from skimage import feature
from skimage import filters

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

img=mpimg.imread('screenshot2.png')


gray = rgb2gray(img)
sobel = filters.roberts_pos_diag(gray)
# Compute the Canny filter for two values of sigma
edges1 = feature.canny(gray)
edges2 = feature.canny(gray, sigma=3)

# display results
fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=1, ncols=4, figsize=(10, 10),
                                    sharex=True, sharey=True)

ax1.imshow(gray, cmap=plt.cm.gray)
ax1.axis('off')
ax1.set_title('noisy image', fontsize=20)

ax2.imshow(edges1, cmap=plt.cm.gray)
ax2.axis('off')
ax2.set_title('Canny filter, $\sigma=1$', fontsize=20)

ax3.imshow(edges2, cmap=plt.cm.gray)
ax3.axis('off')
ax3.set_title('Canny filter, $\sigma=3$', fontsize=20)

ax4.imshow(sobel, cmap=plt.cm.gray)
ax4.axis("off")
ax4.set_title('Sobel filter, fontsize=20')

fig.tight_layout()

#plt.show()


plt.figure(2)
plt.imshow(sobel)
plt.show()