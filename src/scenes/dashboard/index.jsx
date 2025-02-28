import { Box, Button, IconButton, Typography, useTheme } from "@mui/material";
import { tokens } from "../../theme";
import { mockTransactions } from "../../data/mockData";
import DownloadOutlinedIcon from "@mui/icons-material/DownloadOutlined";
import EmailIcon from "@mui/icons-material/Email";
import PointOfSaleIcon from "@mui/icons-material/PointOfSale";
import PersonAddIcon from "@mui/icons-material/PersonAdd";
import TrafficIcon from "@mui/icons-material/Traffic";
import Header from "../../components/Header";
import LineChart from "../../components/LineChart";
import GeographyChart from "../../components/GeographyChart";
import BarChart from "../../components/BarChart";
import StatBox from "../../components/StatBox";
import ProgressCircle from "../../components/ProgressCircle";
import DeleteIcon from '@mui/icons-material/Delete';
import DeleteOutlineIcon from '@mui/icons-material/DeleteOutline';
import DeleteForeverIcon from '@mui/icons-material/DeleteForever';
import RestoreFromTrashIcon from '@mui/icons-material/RestoreFromTrash';
import { useState, useEffect } from 'react';

const Dashboard = () => {
  const theme = useTheme();
  const colors = tokens(theme.palette.mode);

  // 垃圾桶数据状态（单位：L）
  const [generalWaste, setGeneralWaste] = useState(18);
  const [plasticWaste, setPlasticWaste] = useState(26);
  const [mentalWaste, setMentalWaste] = useState(12);
  const [paperWaste, setPaperWaste] = useState(6);

  // 交易记录状态
  const [transactions, setTransactions] = useState([]);

  // **获取垃圾桶容量数据**
  const fetchWasteData = async () => {
    try {
      const response = await fetch("http://192.168.0.105:5000/api/barchartdata");
      const result = await response.json();

      setGeneralWaste(result.find(item => item.bin_type === "General")?.general || 0);
      setPlasticWaste(result.find(item => item.bin_type === "Plastic")?.plastic || 0);
      setMentalWaste(result.find(item => item.bin_type === "Mental")?.mental || 0);
      setPaperWaste(result.find(item => item.bin_type === "Paper")?.paper || 0);
    } catch (error) {
      console.error("Error fetching waste data:", error);
    }
  };

  // **获取垃圾桶状态（是否满了）**
  const fetchBinStatus = async () => {
    try {
      const response = await fetch("http://192.168.0.105:5000/api/bin-status");
      const data = await response.json();

      if (data.bin_full == "Bin is full") {
        console.log("Bin is full! Adding new transaction...");

        const newTransaction = {
          txId: Math.random().toString(36).substring(7), // 生成随机 ID
          user: "General Waste Bin",
          date: new Date().toLocaleString(),
        };

        setTransactions((prev) => [newTransaction, ...prev.slice(0, 8)]); // 只保留最近 5 条记录
      }
    } catch (error) {
      console.error("Error fetching bin status:", error);
    }
  };

  // **统一定时轮询（5 秒一次）**
  useEffect(() => {
    fetchWasteData();  // 获取垃圾桶容量
    fetchBinStatus();   // 获取垃圾桶状态
    const interval = setInterval(() => {
      fetchWasteData();
      fetchBinStatus();
    }, 5000);

    return () => clearInterval(interval); // 组件卸载时清除定时器
  }, []);

  return (
    <Box m="20px">
      {/* HEADER */}
      <Box display="flex" justifyContent="space-between" alignItems="center">
        <Header title="DASHBOARD" subtitle="Trash Level"/>

        <Box>
          <Button
            sx={{
              backgroundColor: colors.blueAccent[700],
              color: colors.grey[100],
              fontSize: "14px",
              fontWeight: "bold",
              padding: "10px 20px",
            }}
          >
            <DownloadOutlinedIcon sx={{ mr: "10px" }} />
            Download Reports
          </Button>
        </Box>
      </Box>

      {/* GRID & CHARTS */}
      <Box
        display="grid"
        gridTemplateColumns="repeat(12, 1fr)"
        gridAutoRows="140px"
        gap="20px"
      >
        {/* ROW 1 */}
        {/* backgroundColor={colors.primary[400]} */}
        <Box
          gridColumn="span 3"
          backgroundColor={colors.grey[300]}
          display="flex"
          alignItems="center"
          justifyContent="center"
        >
          <StatBox
            title={`${generalWaste}L`}
            subtitle="General Waste"
            progress="0.75"
            increase="+14%"
            icon={
              <DeleteIcon
                sx={{ color: colors.greenAccent[600], fontSize: "26px" }}
              />
            }
          />
        </Box>
        <Box
          gridColumn="span 3"
          backgroundColor={colors.greenAccent[500]}
          display="flex"
          alignItems="center"
          justifyContent="center"
        >
          <StatBox
            title={`${plasticWaste}L`}
            subtitle="Plastic"
            progress="0.50"
            increase="+21%"
            icon={
              <DeleteIcon
                sx={{ color: colors.greenAccent[600], fontSize: "26px" }}
              />
            }
          />
        </Box>
        <Box
          gridColumn="span 3"
          backgroundColor={colors.yellowAccent[300]}
          display="flex"
          alignItems="center"
          justifyContent="center"
        >
          <StatBox
            title={`${mentalWaste}L`}
            subtitle="Mental"
            progress="0.30"
            increase="+5%"
            icon={
              <DeleteIcon
                sx={{ color: colors.greenAccent[600], fontSize: "26px" }}
              />
            }
          />
        </Box>
        <Box
          gridColumn="span 3"
          backgroundColor={colors.blueAccent1[700]}
          display="flex"
          alignItems="center"
          justifyContent="center"
        >
          <StatBox
            title={`${paperWaste}L`}
            subtitle="Paper"
            progress="0.80"
            increase="+43%"
            icon={
              <DeleteIcon
                sx={{ color: colors.greenAccent[600], fontSize: "26px" }}
              />
            }
          />
        </Box>

        {/* ROW 2 */}
        <Box
          gridColumn="span 8"
          gridRow="span 2"
          backgroundColor={colors.primary[400]}
        >
          <Box
            mt="25px"
            p="0 30px"
            display="flex "
            justifyContent="space-between"
            alignItems="center"
          >
            <Box>
              <Typography
                variant="h5"
                fontWeight="600"
                color={colors.grey[100]}
              >
                Trash Volume Changes in Four Bins
              </Typography>
              <Typography
                variant="h3"
                fontWeight="bold"
                color={colors.greenAccent[500]}
              >
                8am - 6pm
              </Typography>
            </Box>
            <Box>
              <IconButton>
                <DownloadOutlinedIcon
                  sx={{ fontSize: "26px", color: colors.greenAccent[500] }}
                />
              </IconButton>
            </Box>
          </Box>
          <Box height="250px" m="-20px 0 0 0">
            <LineChart isDashboard={true} />
          </Box>
        </Box>
        <Box
          gridColumn="span 4"
          gridRow="span 2"
          backgroundColor={colors.primary[400]}
        >
          <Typography
            variant="h5"
            fontWeight="600"
            sx={{ padding: "30px 30px 0 30px" }}
          >
            Trash Volume
          </Typography>
          <Box height="250px" mt="-20px">
            <BarChart isDashboard={true} />
          </Box>
        </Box>

        {/* ROW 3 */}
        
        <Box
          gridColumn="span 6"
          gridRow="span 2"
          backgroundColor={colors.primary[400]}
          p="30px"
        >
          <Typography variant="h5" fontWeight="600">
            Total waste disposed
          </Typography>
          <Box
            display="flex"
            flexDirection="column"
            alignItems="center"
            mt="25px"
          >
            <ProgressCircle size="125" />
            <Typography
              variant="h5"
              color={colors.greenAccent[500]}
              sx={{ mt: "15px" }}
            >
              2569L waste disposed
            </Typography>
            <Typography>Includes general, plastic, mental, paper waste</Typography>
          </Box>
        </Box>
        {/* trash volume */}

        <Box
          gridColumn="span 6"
          gridRow="span 2"
          backgroundColor={colors.primary[400]}
          overflow="auto"
        >
          <Box
            display="flex"
            justifyContent="space-between"
            alignItems="center"
            borderBottom={`4px solid ${colors.primary[500]}`}
            colors={colors.grey[100]}
            p="15px"
          >
            <Typography color={colors.grey[100]} variant="h5" fontWeight="600">
              Recent Auto-Waste Disposal
            </Typography>
          </Box>
          {transactions.map((transaction, i) => (
            <Box
              key={`${transaction.txId}-${i}`}
              display="flex"
              justifyContent="space-between"
              alignItems="center"
              borderBottom={`4px solid ${colors.primary[500]}`}
              p="15px"
            >
              <Box>
                <Typography
                  color={colors.greenAccent[500]}
                  variant="h5"
                  fontWeight="600"
                >
                  {transaction.txId}
                </Typography>
                <Typography color={colors.grey[100]}>
                  {transaction.user}
                </Typography>
              </Box>
              {/* <Box color={colors.grey[100]}>{transaction.date}</Box> */}
              <Box sx={{ color: colors.grey[100], mr: 2 }}>{transaction.date}</Box>
              {/* <Box
                backgroundColor={colors.greenAccent[500]}
                p="5px 10px"
                borderRadius="4px"
              >
                ${transaction.cost}
              </Box> */}
            </Box>
          ))}
        </Box>
        
        {/* <Box
          gridColumn="span 4"
          gridRow="span 2"
          backgroundColor={colors.primary[400]}
          padding="30px"
        >
          <Typography
            variant="h5"
            fontWeight="600"
            sx={{ marginBottom: "15px" }}
          >
            Geography Based Traffic
          </Typography>
          <Box height="200px">
            <GeographyChart isDashboard={true} />
          </Box>
        </Box> */}
      </Box>
    </Box>
  );
};

export default Dashboard;
