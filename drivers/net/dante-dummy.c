// SPDX-License-Identifier: GPL-2.0-only

#include <linux/of_platform.h>
#include <linux/workqueue.h>
#include <linux/phylink.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/rtnetlink.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phylink.h>

#define DRV_NAME	"dummy"

struct dummy_net_priv {
	struct device		*dev;
	struct net_device	*ndev;

	struct phylink		*phylink;
	struct phylink_config	phylink_config;
};

static void dummy_set_multicast_list(struct net_device *dev)
{
}

static netdev_tx_t dummy_xmit(struct sk_buff *skb, struct net_device *dev)
{
	skb_tx_timestamp(skb);
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int dummy_dev_init(struct net_device *dev)
{
	return 0;
}

static int dummy_change_carrier(struct net_device *dev, bool new_carrier)
{
	if (new_carrier)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);

	return 0;
}

static int dummy_net_open(struct net_device *ndev)
{
	struct dummy_net_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	int ret;

	ret = phylink_of_phy_connect(priv->phylink, dev->of_node, 0);
	if (ret) {
		dev_err(dev, "phylink_of_phy_connect() failed: %d\n", ret);
		return ret;
	}

	phylink_start(priv->phylink);

	return 0;
}

static int dummy_net_stop(struct net_device *ndev)
{
	struct dummy_net_priv *priv = netdev_priv(ndev);

	phylink_stop(priv->phylink);
	phylink_disconnect_phy(priv->phylink);

	netif_stop_queue(ndev);

	return 0;
}

static const struct net_device_ops dummy_netdev_ops = {
	.ndo_init		= dummy_dev_init,
	.ndo_open		= dummy_net_open,
	.ndo_stop		= dummy_net_stop,
	.ndo_start_xmit		= dummy_xmit,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_rx_mode	= dummy_set_multicast_list,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_carrier	= dummy_change_carrier,
};

static const struct ethtool_ops dummy_ethtool_ops = {
	.get_ts_info		= ethtool_op_get_ts_info,
};

static void dummy_net_phylink_destroy(void *phylink)
{
	phylink_destroy(phylink);
}

static void dummy_net_validate(struct phylink_config *config,
			       unsigned long *supported,
			       struct phylink_link_state *state)
{
}

static void dummy_net_mac_config(struct phylink_config *config, unsigned int mode,
				 const struct phylink_link_state *state)
{
}

static void dummy_net_mac_link_down(struct phylink_config *config,
				    unsigned int mode,
				    phy_interface_t interface)
{
}

static void dummy_net_mac_link_up(struct phylink_config *config,
				  struct phy_device *phy,
				  unsigned int mode, phy_interface_t interface,
				  int speed, int duplex,
				  bool tx_pause, bool rx_pause)
{
}

static const struct phylink_mac_ops dummy_net_phylink_ops = {
	.validate		= dummy_net_validate,
	.mac_config		= dummy_net_mac_config,
	.mac_link_down		= dummy_net_mac_link_down,
	.mac_link_up		= dummy_net_mac_link_up,
};

static int dummy_net_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dummy_net_priv *priv;
	struct net_device *ndev;
	phy_interface_t phy_mode;
	int ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;

	platform_set_drvdata(pdev, ndev);

	strcpy(ndev->name, "da%d");
	SET_NETDEV_DEV(ndev, dev);

	ether_setup(ndev);

	ndev->netdev_ops = &dummy_netdev_ops;
	ndev->ethtool_ops = &dummy_ethtool_ops;
	ndev->needs_free_netdev = true;

	ndev->flags |= IFF_NOARP;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->priv_flags |= IFF_LIVE_ADDR_CHANGE | IFF_NO_QUEUE;
	ndev->features	|= NETIF_F_SG | NETIF_F_FRAGLIST;
	ndev->features	|= NETIF_F_GSO_SOFTWARE;
	ndev->features	|= NETIF_F_HW_CSUM | NETIF_F_HIGHDMA | NETIF_F_LLTX;
	ndev->features	|= NETIF_F_GSO_ENCAP_ALL;
	ndev->hw_features |= ndev->features;
	ndev->hw_enc_features |= ndev->features;
	eth_hw_addr_random(ndev);

	ndev->min_mtu = 0;
	ndev->max_mtu = 0;

	/* Phylink */
	phy_mode = PHY_INTERFACE_MODE_RGMII;
	of_get_phy_mode(dev->of_node, &phy_mode);

	priv->phylink_config.dev = &ndev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.mac_capabilities = MAC_10 | MAC_100 | MAC_1000;

	__set_bit(PHY_INTERFACE_MODE_RGMII, priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_RGMII_ID, priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_RGMII_RXID, priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_RGMII_TXID, priv->phylink_config.supported_interfaces);

	priv->phylink = phylink_create(&priv->phylink_config, dev->fwnode,
				       phy_mode, &dummy_net_phylink_ops);
	if (IS_ERR(priv->phylink))
		return PTR_ERR(priv->phylink);

	ret = devm_add_action_or_reset(dev, dummy_net_phylink_destroy, priv->phylink);
	if (ret < 0)
		return ret;

	ret = devm_register_netdev(dev, ndev);
	if (ret < 0) {
		dev_err(dev, "could not register network device: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id dummy_net_of_ids[] = {
	{ .compatible = "holoplot,dante-dummy" },
	{}
};
MODULE_DEVICE_TABLE(of, dummy_net_of_ids);

static struct platform_driver dummy_net_driver =
{
	.probe = dummy_net_probe,
	.driver = {
		.name = "dummy_net_driver",
		.of_match_table = dummy_net_of_ids,
	},
};

module_platform_driver(dummy_net_driver);

MODULE_DESCRIPTION("Dante dummy net driver");
MODULE_LICENSE("GPL");
